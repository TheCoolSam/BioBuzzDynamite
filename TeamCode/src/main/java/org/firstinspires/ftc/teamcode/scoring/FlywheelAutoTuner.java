package org.firstinspires.ftc.teamcode.scoring;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Hands-off flywheel PIDF tuning. The robot grades its own tuning from the flywheel encoder.
 *
 * <ol>
 *   <li><b>F sweep</b>: open loop at three powers. Each steady speed (normalised to 12 V) gives one
 *       point of power vs speed; a straight-line fit gives the friction floor and the power per
 *       tick/s, and from those the F that holds the target speed, plus the top speed.</li>
 *   <li><b>P search</b>: with that F, settle 15 % below target, step to target, and score the rise
 *       time and overshoot for several P values. Prefer a fast step with at most 3 % overshoot
 *       and little variation once settled.</li>
 *   <li><b>I check</b>: hold the target with I = 1.2 F and read the leftover error. Too much error
 *       doubles I; wobbling halves it. Accept the result only if the final hold meets both limits.</li>
 * </ol>
 *
 * All gains are in the hub's units (see {@code FlywheelPair}): power = (F * target + P * error) / 32767
 * at 12 V, plus the I term. Speeds are rpm. Call {@link #update} every loop with the filtered speed.
 */
public final class FlywheelAutoTuner {
    public enum Phase { IDLE, F_SWEEP, P_SEARCH, I_CHECK, DONE, FAILED }

    /** What the flywheel should do this loop. */
    public static final class Output {
        public final boolean on, velocityMode;
        public final double power, targetRpm, p, i, f;
        Output(boolean on, boolean velocityMode, double power, double targetRpm, double p, double i, double f) {
            this.on = on; this.velocityMode = velocityMode; this.power = power; this.targetRpm = targetRpm;
            this.p = p; this.i = i; this.f = f;
        }
    }

    public static final double HUB_FULL_SCALE = 32767.0;
    static final double[] SWEEP_POWERS = {0.45, 0.65, 0.85};
    static final double[] P_MULTIPLIERS = {0.6, 1.2, 2.4, 4.8};
    static final double SWEEP_WINDOW_SEC = 1.25, SWEEP_TIMEOUT_SEC = 15.0;
    static final double SWEEP_MAX_DRIFT = 0.015, SWEEP_MAX_VARIATION = 0.025;
    static final double PRE_STEP_FRACTION = 0.85, PRE_SETTLE_SEC = 2.0, STEP_WATCH_SEC = 2.0;
    static final double MAX_OVERSHOOT = 0.03, MAX_WOBBLE = 0.01, I_HOLD_SEC = 4.0;
    static final double MAX_HOLD_ERROR_PCT = 1.0, MAX_HOLD_WOBBLE_PCT = 1.5;
    static final double NO_SPIN_RPM = 100.0, NO_SPIN_SEC = 2.0;

    private final double targetRpm, ticksPerRev, maxSafeRpm;
    private Phase phase = Phase.IDLE;
    private String message = "Set AUTO_TUNE_PIDF in Dashboard";

    // F sweep
    private int sweepIndex;
    private double stepStart, anchorRpm = Double.NaN, anchorSince;
    private final List<double[]> sweep = new ArrayList<double[]>(); // {rpm, power at 12 V}
    private double sweepElapsedSec, sweepWindowStart, previousSweepRpm;
    private double sweepRpmSum, sweepRpmSquaredSum, sweepVoltsSum;
    private int sweepSamples;
    private double kS = Double.NaN, kVPerTps = Double.NaN, f = Double.NaN, topRpm = Double.NaN;

    // P search
    private int pIndex;
    private boolean stepped;
    private double stepAt, riseSec, peakRpm;
    private final List<double[]> stepSamples = new ArrayList<double[]>(); // {t, rpm} after the step
    private final List<String> pReport = new ArrayList<String>();
    private double bestP = Double.NaN, bestRise = Double.NaN;
    private double fallbackP = Double.NaN, fallbackError;

    // I check
    private double iGain = Double.NaN, holdStart;
    private int iTries;
    private final List<Double> holdSamples = new ArrayList<Double>();
    private double holdErrorPct = Double.NaN, holdWobblePct = Double.NaN;

    private double noSpinSince = Double.NaN;

    public FlywheelAutoTuner(double targetRpm, double ticksPerRev, double maxSafeRpm) {
        if (!(targetRpm >= 200 && targetRpm < maxSafeRpm) || !Double.isFinite(maxSafeRpm)
                || !(ticksPerRev > 0) || !Double.isFinite(ticksPerRev)) {
            throw new IllegalArgumentException("Set TARGET_RPM from 200 to below the RPM limit and check TICKS_PER_REV");
        }
        this.targetRpm = targetRpm;
        this.ticksPerRev = ticksPerRev;
        this.maxSafeRpm = maxSafeRpm;
    }

    public Phase phase() { return phase; }
    public boolean running() { return phase == Phase.F_SWEEP || phase == Phase.P_SEARCH || phase == Phase.I_CHECK; }
    public String message() { return message; }
    public List<String> pReport() { return pReport; }
    public double targetRpm() { return targetRpm; }
    public double sweepElapsedSec() { return sweepElapsedSec; }
    public double resultP() { return bestP; }
    public double resultI() { return iGain; }
    public double resultF() { return f; }
    public double topRpm() { return topRpm; }
    public double riseSec() { return bestRise; }
    public double holdErrorPct() { return holdErrorPct; }

    public void start(double now) {
        phase = Phase.F_SWEEP;
        sweepIndex = 0;
        sweep.clear();
        pReport.clear();
        f = Double.NaN;
        topRpm = Double.NaN;
        bestP = Double.NaN;
        fallbackP = Double.NaN;
        fallbackError = Double.POSITIVE_INFINITY;
        iGain = Double.NaN;
        holdErrorPct = Double.NaN;
        holdWobblePct = Double.NaN;
        noSpinSince = Double.NaN;
        startSweepStep(now);
    }

    public void abort(String why) {
        if (running()) { phase = Phase.FAILED; message = "Stopped: " + why; }
    }

    /**
     * One step. {@code rpm} is the filtered flywheel speed, {@code volts} the battery voltage.
     * Returns what to command; when not running the flywheel is off.
     */
    public Output update(double now, double rpm, double volts) {
        if (!running()) return off();
        if (!Double.isFinite(rpm)) { fail("encoder speed reading is invalid"); return off(); }
        if (rpm < -NO_SPIN_RPM) { fail("encoder speed is negative (check motor direction)"); return off(); }
        if (rpm > maxSafeRpm) { fail(String.format(Locale.US, "speed %.0f rpm passed the safe limit %.0f", rpm, maxSafeRpm)); return off(); }
        if (!(volts > 7 && volts < 15.5)) { fail("battery voltage reading is invalid"); return off(); }
        if (Double.isFinite(f) && !checkHeadroom(volts)) return off();
        if (rpm < NO_SPIN_RPM) {
            if (Double.isNaN(noSpinSince)) noSpinSince = now;
            if (now - noSpinSince > NO_SPIN_SEC) {
                fail("flywheel not spinning (check motor, wiring and encoder)");
                return off();
            }
        } else {
            noSpinSince = Double.NaN;
        }
        switch (phase) {
            case F_SWEEP: return sweepStep(now, rpm, volts);
            case P_SEARCH: return pStep(now, rpm);
            case I_CHECK: return iStep(now, rpm);
            default: return off();
        }
    }

    // ------------------------------------------------------------------ F sweep
    private Output sweepStep(double now, double rpm, double volts) {
        double power = SWEEP_POWERS[sweepIndex];
        sweepElapsedSec = now - stepStart;
        if (sweepElapsedSec > SWEEP_TIMEOUT_SEC) {
            fail(String.format(Locale.US, "speed did not settle at %s power after %.1f s (last reading %.0f rpm, %.1f V)",
                    pct(power), sweepElapsedSec, rpm, volts));
            return off();
        }
        if (collectSweepPoint(now, rpm, volts, power)) {
            sweepIndex++;
            if (sweepIndex >= SWEEP_POWERS.length) {
                if (!fitF() || !checkHeadroom(volts)) return off();
                phase = Phase.P_SEARCH;
                pIndex = 0;
                startTrial(now);
                return velocity(PRE_STEP_FRACTION * targetRpm, P_MULTIPLIERS[0] * f, 0);
            }
            startSweepStep(now);
            power = SWEEP_POWERS[sweepIndex];
        }
        return new Output(true, false, power, 0, Double.NaN, Double.NaN, Double.NaN);
    }

    private void startSweepStep(double now) {
        stepStart = now;
        sweepElapsedSec = 0;
        previousSweepRpm = Double.NaN;
        resetSweepWindow(now);
        message = "F sweep: open loop at " + pct(SWEEP_POWERS[sweepIndex]);
    }

    private void resetSweepWindow(double now) {
        sweepWindowStart = now;
        sweepSamples = 0;
        sweepRpmSum = 0;
        sweepRpmSquaredSum = 0;
        sweepVoltsSum = 0;
    }

    /** Wait for two consecutive low-variation windows with nearly the same mean speed. */
    private boolean collectSweepPoint(double now, double rpm, double volts, double power) {
        if (now <= sweepWindowStart) return false;
        sweepSamples++;
        sweepRpmSum += rpm;
        sweepRpmSquaredSum += rpm * rpm;
        sweepVoltsSum += volts;
        if (now - sweepWindowStart < SWEEP_WINDOW_SEC) return false;

        double meanRpm = sweepRpmSum / sweepSamples;
        double variance = Math.max(0, sweepRpmSquaredSum / sweepSamples - meanRpm * meanRpm);
        boolean quiet = sweepSamples >= 5 && meanRpm >= NO_SPIN_RPM
                && Math.sqrt(variance) <= SWEEP_MAX_VARIATION * meanRpm;
        boolean settled = quiet && Math.abs(meanRpm - previousSweepRpm) <= SWEEP_MAX_DRIFT * meanRpm;
        if (settled) sweep.add(new double[]{meanRpm, power * (sweepVoltsSum / sweepSamples) / 12.0});
        previousSweepRpm = quiet ? meanRpm : Double.NaN;
        resetSweepWindow(now);
        return settled;
    }

    private boolean fitF() {
        int n = sweep.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] s : sweep) {
            double tps = s[0] * ticksPerRev / 60.0;
            sx += tps; sy += s[1]; sxx += tps * tps; sxy += tps * s[1];
        }
        double den = n * sxx - sx * sx;
        if (!(Math.abs(den) > 1e-9)) { fail("speeds did not change between power levels"); return false; }
        kVPerTps = (n * sxy - sx * sy) / den;
        kS = (sy - kVPerTps * sx) / n;
        if (!(kVPerTps > 0)) { fail("speed went down as power went up (check the encoder direction)"); return false; }
        double targetTps = targetRpm * ticksPerRev / 60.0;
        f = HUB_FULL_SCALE * (kS + kVPerTps * targetTps) / targetTps;
        topRpm = (1.0 - kS) / kVPerTps * 60.0 / ticksPerRev;
        if (!(f > 0) || !Double.isFinite(f) || !Double.isFinite(topRpm)) {
            fail("power sweep did not produce a usable feedforward");
            return false;
        }
        return true;
    }

    private boolean checkHeadroom(double volts) {
        double topAtBattery = (volts / 12.0 - kS) / kVPerTps * 60.0 / ticksPerRev;
        if (targetRpm <= 0.9 * topAtBattery) return true;
        fail(String.format(Locale.US,
                "target %.0f rpm is above 90%% of the estimated top speed %.0f rpm at %.1f V. Lower TARGET_RPM.",
                targetRpm, topAtBattery, volts));
        return false;
    }

    // ------------------------------------------------------------------ P search
    private void startTrial(double now) {
        stepped = false;
        stepStart = now;
        anchorRpm = Double.NaN;
        stepSamples.clear();
        riseSec = Double.NaN;
        peakRpm = 0;
        message = String.format(Locale.US, "P search %d/%d: P = %.1f", pIndex + 1, P_MULTIPLIERS.length, P_MULTIPLIERS[pIndex] * f);
    }

    private Output pStep(double now, double rpm) {
        double p = P_MULTIPLIERS[pIndex] * f;
        if (!stepped) {
            double pre = PRE_STEP_FRACTION * targetRpm;
            boolean near = Math.abs(rpm - pre) < 0.03 * pre;
            if (near && Double.isNaN(anchorRpm)) { anchorRpm = rpm; anchorSince = now; }
            if (!near) anchorRpm = Double.NaN;
            if (near && now - anchorSince >= PRE_SETTLE_SEC) {
                stepped = true;
                stepAt = now;
                return velocity(targetRpm, p, 0);
            }
            if (now - stepStart > 8.0) {
                pReport.add(String.format(Locale.US, "P %.1f: could not settle before the step", p));
                return nextTrial(now);
            }
            return velocity(pre, p, 0);
        }
        double t = now - stepAt;
        stepSamples.add(new double[]{t, rpm});
        peakRpm = Math.max(peakRpm, rpm);
        if (Double.isNaN(riseSec) && rpm >= 0.98 * targetRpm) riseSec = t;
        if (t < STEP_WATCH_SEC) return velocity(targetRpm, p, 0);
        // score this P
        double overshoot = Math.max(0, peakRpm - targetRpm) / targetRpm;
        double wobble = wobble(stepSamples, STEP_WATCH_SEC / 2);
        boolean stable = overshoot <= MAX_OVERSHOOT && wobble <= MAX_WOBBLE;
        double error = Math.abs(targetRpm - rpm) / targetRpm;
        boolean ok = stable && !Double.isNaN(riseSec) && error <= 0.02;
        pReport.add(String.format(Locale.US, "P %.1f: rise %s, overshoot %.1f%%, wobble %.1f%% %s", p,
                Double.isNaN(riseSec) ? "never" : String.format(Locale.US, "%.2f s", riseSec), overshoot * 100, wobble * 100, ok ? "OK" : "x"));
        if (ok && (Double.isNaN(bestP) || riseSec < bestRise - 0.05)) {
            bestP = p;
            bestRise = riseSec;
        }
        // Allow some remaining P-only error if the I check can remove it and pass the final hold.
        if (stable && error <= 0.05 && error < fallbackError) {
            fallbackP = p;
            fallbackError = error;
        }
        return nextTrial(now);
    }

    private Output nextTrial(double now) {
        pIndex++;
        if (pIndex < P_MULTIPLIERS.length) { startTrial(now); return velocity(PRE_STEP_FRACTION * targetRpm, P_MULTIPLIERS[pIndex] * f, 0); }
        if (Double.isNaN(bestP)) {
            if (Double.isNaN(fallbackP)) { fail("no P candidate held a stable speed near the target"); return off(); }
            bestP = fallbackP;
            bestRise = Double.NaN;
        }
        phase = Phase.I_CHECK;
        iGain = 1.2 * f;
        iTries = 0;
        startHold(now);
        return velocity(targetRpm, bestP, iGain);
    }

    // ------------------------------------------------------------------ I check
    private void startHold(double now) {
        holdStart = now;
        holdSamples.clear();
        message = String.format(Locale.US, "I check: holding %.0f rpm with I = %.1f", targetRpm, iGain);
    }

    private Output iStep(double now, double rpm) {
        double t = now - holdStart;
        if (t >= I_HOLD_SEC / 2) holdSamples.add(rpm);
        if (t < I_HOLD_SEC) return velocity(targetRpm, bestP, iGain);
        if (holdSamples.size() < 5) {
            fail("too few encoder samples to validate the speed hold");
            return off();
        }
        double mean = 0;
        for (double s : holdSamples) mean += s;
        mean /= holdSamples.size();
        double var = 0;
        for (double s : holdSamples) var += (s - mean) * (s - mean);
        holdErrorPct = 100 * Math.abs(mean - targetRpm) / targetRpm;
        holdWobblePct = 100 * Math.sqrt(var / holdSamples.size()) / targetRpm;
        boolean wobbly = holdWobblePct > MAX_HOLD_WOBBLE_PCT;
        if (wobbly || holdErrorPct > MAX_HOLD_ERROR_PCT) {
            if (iTries >= 2) {
                fail(String.format(Locale.US, "speed hold exceeded limits (%.1f%% error, %.1f%% variation)", holdErrorPct, holdWobblePct));
                return off();
            }
            iGain *= wobbly ? 0.5 : 2.0;
            iTries++;
            startHold(now);
            return velocity(targetRpm, bestP, iGain);
        }
        phase = Phase.DONE;
        message = String.format(Locale.US,
                "Done. Hold error %.1f%%, variation %.1f%%. Estimated top speed at 12 V %.0f rpm. Use SAVE_CALIBRATION.",
                holdErrorPct, holdWobblePct, topRpm);
        return off();
    }

    // ------------------------------------------------------------------ helpers
    /** Standard deviation of the samples after {@code fromSec}, as a fraction of the target. */
    private double wobble(List<double[]> samples, double fromSec) {
        double sum = 0, sq = 0;
        int n = 0;
        for (double[] s : samples) if (s[0] >= fromSec) { sum += s[1]; sq += s[1] * s[1]; n++; }
        if (n < 5) return Double.POSITIVE_INFINITY;
        double mean = sum / n;
        return Math.sqrt(Math.max(0, sq / n - mean * mean)) / targetRpm;
    }

    private void fail(String why) { phase = Phase.FAILED; message = "FAILED: " + why; }

    private Output velocity(double rpm, double p, double i) { return new Output(true, true, 0, rpm, p, i, f); }

    private static Output off() { return new Output(false, false, 0, 0, Double.NaN, Double.NaN, Double.NaN); }

    private static String pct(double power) { return String.format(Locale.US, "%.0f%%", power * 100); }
}
