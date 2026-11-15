package org.firstinspires.ftc.teamcode.turret;

/**
 * Camera-driven yaw aiming: turns a Limelight horizontal offset (tx, degrees) into a normalized
 * turret yaw demand in [-maxPower, +maxPower]. The goal is tx == aimOffsetDeg (usually 0).
 *
 * <p>Pure logic. No SDK or hardware access, so it runs on the desktop and in OpModes. It never
 * drives hardware itself: the caller passes the demand through the turret output safety gate
 * ({@code RevTurretIO.command}), which owns limits, arming, deadman and e-stop.
 *
 * <p>Assumes the camera rides on the turret, so tx == 0 means the shooter points at the target.
 * If the camera is fixed to the chassis this is NOT the right controller.
 *
 * <p>Behavior:
 * <ul>
 *   <li>tx is low-pass filtered; the D term uses the filtered rate, so Limelight frame jitter does
 *       not become power jitter.</li>
 *   <li>Inside {@link Config#deadbandDeg} the demand is exactly zero (no hunting).</li>
 *   <li>With no fresh target the demand is zero immediately (hold position, never chase stale data),
 *       and the filter resets after {@link Config#lostResetSec}.</li>
 *   <li>{@link Output#locked} needs {@link Config#lockFrames} consecutive in-tolerance frames.</li>
 * </ul>
 */
public final class VisionAimController {

    public static final class Config {
        /** Power per degree of error. */
        public double kP = 0.012;
        /** Power per (degree/second) of filtered error rate. Damps overshoot. */
        public double kD = 0.0004;
        /** Demand cap. The output safety gate applies its own, possibly lower, cap. */
        public double maxPower = 0.15;
        /** Below this |error| the demand is zero. */
        public double deadbandDeg = 0.5;
        /** |error| at or below this counts toward lock. */
        public double lockDeg = 1.0;
        public int lockFrames = 5;
        /** 0..1. Higher follows tx faster but passes more noise. */
        public double filterAlpha = 0.5;
        /** Seconds without a target before the filter and derivative memory reset. */
        public double lostResetSec = 0.3;
        /** +1 or -1. Flip if positive power turns the turret away from positive tx. */
        public int sign = 1;
        /** Desired tx. Trims camera/shooter misalignment. */
        public double aimOffsetDeg = 0.0;

        public Config copy() {
            Config c = new Config();
            c.kP = kP; c.kD = kD; c.maxPower = maxPower; c.deadbandDeg = deadbandDeg;
            c.lockDeg = lockDeg; c.lockFrames = lockFrames; c.filterAlpha = filterAlpha;
            c.lostResetSec = lostResetSec; c.sign = sign; c.aimOffsetDeg = aimOffsetDeg;
            return c;
        }
    }

    public static final class Output {
        public double demand;
        public boolean targetVisible;
        public boolean locked;
        public double filteredTxDeg = Double.NaN;
        public double errorDeg = Double.NaN;
        public double errorRateDegPerSec;
        /** SEARCHING, TRACKING, IN_DEADBAND, LOCKED or BAD_CONFIG. */
        public String state = "SEARCHING";
    }

    private Config config = new Config();
    private double filtered = Double.NaN;
    private double lastFiltered = Double.NaN;
    private double rate;
    private double lostSec;
    private int lockCount;

    public VisionAimController() { }

    public VisionAimController(Config config) { setConfig(config); }

    public void setConfig(Config config) {
        if (config == null) throw new IllegalArgumentException("config required");
        this.config = config.copy();
    }

    public void reset() {
        filtered = Double.NaN;
        lastFiltered = Double.NaN;
        rate = 0;
        lostSec = 0;
        lockCount = 0;
    }

    /**
     * @param visible true only for a FRESH detection of the wanted target
     * @param txDeg   horizontal offset in degrees, positive = target right of the crosshair
     * @param dt      loop time in seconds
     */
    public Output update(boolean visible, double txDeg, double dt) {
        Output out = new Output();
        Config c = config;
        if (!validConfig(c)) {
            out.state = "BAD_CONFIG";
            reset();
            return out;
        }
        double step = Double.isFinite(dt) ? Math.max(1e-3, Math.min(0.2, dt)) : 0.02;

        if (!visible || !Double.isFinite(txDeg)) {
            lostSec += step;
            lockCount = 0;
            if (lostSec >= c.lostResetSec) reset();
            out.state = "SEARCHING";
            return out; // demand 0: hold position
        }
        lostSec = 0;
        out.targetVisible = true;

        if (Double.isNaN(filtered)) {
            filtered = txDeg;
            lastFiltered = txDeg;
            rate = 0;
        } else {
            filtered += c.filterAlpha * (txDeg - filtered);
            double instantRate = (filtered - lastFiltered) / step;
            rate += 0.5 * (instantRate - rate);
            lastFiltered = filtered;
        }

        double error = filtered - c.aimOffsetDeg;
        out.filteredTxDeg = filtered;
        out.errorDeg = error;
        out.errorRateDegPerSec = rate;

        if (Math.abs(error) <= c.lockDeg) lockCount++; else lockCount = 0;
        out.locked = lockCount >= c.lockFrames;

        if (Math.abs(error) <= c.deadbandDeg) {
            out.state = out.locked ? "LOCKED" : "IN_DEADBAND";
            return out;
        }
        double raw = c.sign * (c.kP * error + c.kD * rate);
        out.demand = Math.max(-c.maxPower, Math.min(c.maxPower, raw));
        out.state = out.locked ? "LOCKED" : "TRACKING";
        return out;
    }

    private static boolean validConfig(Config c) {
        return finite(c.kP) && c.kP >= 0 && finite(c.kD) && c.kD >= 0
                && finite(c.maxPower) && c.maxPower > 0 && c.maxPower <= 1
                && finite(c.deadbandDeg) && c.deadbandDeg >= 0
                && finite(c.lockDeg) && c.lockDeg >= c.deadbandDeg && c.lockFrames >= 1
                && finite(c.filterAlpha) && c.filterAlpha > 0 && c.filterAlpha <= 1
                && finite(c.lostResetSec) && c.lostResetSec >= 0
                && (c.sign == 1 || c.sign == -1) && finite(c.aimOffsetDeg);
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
