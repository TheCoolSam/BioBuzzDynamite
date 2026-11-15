package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * Decides when the flywheel is fast enough to feed a ball. Pure logic, no SDK.
 *
 * <ul>
 *   <li>VELOCITY: ready when rpm is within {@code tolerance} of the target (closed loop).</li>
 *   <li>OPEN_LOOP: there is no target rpm. Learn the plateau: once rpm stops changing (steady) that
 *       value becomes the baseline. After a ball dips the speed, ready again when rpm is back within
 *       {@code recoverFraction} of the baseline.</li>
 *   <li>TIMED: no usable encoder. Ready a fixed time after the flywheel was switched on.</li>
 * </ul>
 * If the encoder never shows speed while the flywheel is on, any mode falls back to TIMED and
 * {@link #noSignal()} becomes true, so a dead encoder cannot hang the auto.
 */
public final class FlywheelReadyDetector {
    public enum Mode { VELOCITY, OPEN_LOOP, TIMED }

    public static final class Config {
        public Mode mode = Mode.OPEN_LOOP;
        public double targetRpm = 3000;
        /** VELOCITY: allowed error as a fraction of target. */
        public double tolerance = 0.03;
        /**
         * OPEN_LOOP: the window counts as steady when its second half averages within this fraction of
         * the first half (no trend). Judging the trend instead of the peak-to-peak spread tolerates the
         * noise in encoder velocity readings.
         */
        public double steadyBand = 0.006;
        /** OPEN_LOOP: sanity limit on peak-to-peak spread inside the window (fraction of mean). */
        public double maxSpread = 0.12;
        public double steadyWindowSec = 0.6;
        /** Below this the flywheel is considered not running. */
        public double minRpm = 300;
        /** OPEN_LOOP: after a dip, ready again at this fraction below baseline. */
        public double recoverFraction = 0.03;
        /** Never ready before this long after switching on, in any mode (avoids a false steady at 0). */
        public double minOnSec = 1.0;
        /** TIMED and fallback: ready after this long. */
        public double timedSpinupSec = 3.0;
        /** The ready test uses the average of this much recent history, so one noisy reading cannot flip it. */
        public double smoothSec = 0.15;
        /** If no speed above minRpm is ever seen by this time, declare no signal. */
        public double noSignalAfterSec = 1.5;
    }

    private static final class Sample {
        final double t, rpm;
        Sample(double t, double rpm) { this.t = t; this.rpm = rpm; }
    }

    private final Config c;
    private final ArrayDeque<Sample> window = new ArrayDeque<Sample>();
    private double onTime;
    private double baseline = Double.NaN;
    private double maxSeen;
    private boolean ready;
    private boolean noSignal;

    public FlywheelReadyDetector(Config config) {
        if (config == null) throw new IllegalArgumentException("config required");
        this.c = config;
    }

    public void reset() {
        window.clear();
        onTime = 0;
        baseline = Double.NaN;
        maxSeen = 0;
        ready = false;
        noSignal = false;
    }

    public boolean ready() { return ready; }
    public boolean noSignal() { return noSignal; }
    public double baselineRpm() { return baseline; }
    public double onSeconds() { return onTime; }

    public void update(double dt, double rpm, boolean flywheelOn) {
        if (!flywheelOn) { reset(); return; }
        if (!(dt >= 0) || Double.isInfinite(dt)) dt = 0;
        onTime += dt;
        if (Double.isNaN(rpm) || Double.isInfinite(rpm)) rpm = 0;
        rpm = Math.abs(rpm);
        maxSeen = Math.max(maxSeen, rpm);

        if (!noSignal && c.mode != Mode.TIMED && onTime >= c.noSignalAfterSec && maxSeen < c.minRpm) noSignal = true;

        if (c.mode == Mode.TIMED || noSignal) {
            ready = onTime >= c.timedSpinupSec;
            return;
        }

        window.addLast(new Sample(onTime, rpm));
        while (window.size() > 1 && onTime - window.peekFirst().t > c.steadyWindowSec) window.removeFirst();

        if (onTime < c.minOnSec) { ready = false; return; }

        double smooth = smoothedRpm();
        if (c.mode == Mode.VELOCITY) {
            ready = c.targetRpm > 0 && Math.abs(smooth - c.targetRpm) <= c.tolerance * c.targetRpm;
            return;
        }

        // OPEN_LOOP. The baseline only ever moves up, so learning it a little early (while the flywheel is
        // still creeping toward its plateau) is corrected as soon as the plateau is seen.
        double mean = steadyMean();
        if (!Double.isNaN(mean) && (Double.isNaN(baseline) || mean > baseline)) baseline = mean;
        ready = !Double.isNaN(baseline) && smooth >= baseline * (1.0 - c.recoverFraction);
    }

    private double smoothedRpm() {
        double sum = 0;
        int n = 0;
        for (Iterator<Sample> it = window.descendingIterator(); it.hasNext(); ) {
            Sample sm = it.next();
            if (onTime - sm.t > c.smoothSec && n > 0) break;
            sum += sm.rpm;
            n++;
        }
        return n == 0 ? 0 : sum / n;
    }

    /** Mean rpm if the window spans long enough and shows no trend, else NaN. */
    private double steadyMean() {
        if (window.size() < 4) return Double.NaN;
        double first = window.peekFirst().t, last = window.peekLast().t;
        if (last - first < c.steadyWindowSec * 0.9) return Double.NaN;
        double mid = (first + last) / 2.0;
        double min = Double.MAX_VALUE, max = 0, sum = 0, sumA = 0, sumB = 0;
        int nA = 0, nB = 0;
        for (Iterator<Sample> it = window.iterator(); it.hasNext(); ) {
            Sample sm = it.next();
            min = Math.min(min, sm.rpm);
            max = Math.max(max, sm.rpm);
            sum += sm.rpm;
            if (sm.t <= mid) { sumA += sm.rpm; nA++; } else { sumB += sm.rpm; nB++; }
        }
        if (nA == 0 || nB == 0) return Double.NaN;
        double mean = sum / window.size();
        if (mean < c.minRpm) return Double.NaN;
        if (max - min > c.maxSpread * mean) return Double.NaN;
        double trend = Math.abs(sumB / nB - sumA / nA);
        return trend <= c.steadyBand * mean ? mean : Double.NaN;
    }
}
