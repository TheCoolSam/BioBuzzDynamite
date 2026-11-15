package org.firstinspires.ftc.teamcode.auto;

/**
 * Keeps the robot pointed the same way while it drives, by turning the heading error into a small yaw
 * command. Pure logic, no SDK.
 *
 * <p>Convention: headings are degrees, positive counterclockwise (the FTC SDK IMU convention) and the
 * returned yaw command is positive counterclockwise too, the same as {@code BasicDrive}. If the robot
 * turns the wrong way when you push it, flip {@link Config#reversed}.
 *
 * <p>Safety: the correction is capped, and if the error ever exceeds {@link Config#faultDeg} the hold
 * latches off for the rest of the run and returns zero. That is what stops a wrongly oriented IMU, a wrong
 * sign, or a collision from turning into a spin.
 */
public final class HeadingHold {
    public static final class Config {
        /** Yaw command per degree of error. */
        public double kP = 0.02;
        /** Largest yaw command ever returned. */
        public double maxCorrection = 0.20;
        /** Errors smaller than this give no correction (stops dithering). */
        public double deadbandDeg = 0.5;
        /** Larger error than this latches the hold off. */
        public double faultDeg = 20.0;
        /** Flip if the IMU's positive direction is clockwise on your robot. */
        public boolean reversed = false;
        /** Consecutive unreadable headings before the hold latches off. */
        public int maxBadReadings = 25;
    }

    private final Config c;
    private double target;
    private boolean targetSet;
    private boolean faulted;
    private String faultReason = "";
    private double lastError;
    private int badReadings;

    public HeadingHold(Config config) {
        if (config == null) throw new IllegalArgumentException("config required");
        if (!finite(config.kP) || config.kP < 0 || !finite(config.maxCorrection) || config.maxCorrection < 0 || config.maxCorrection > 1
                || !finite(config.deadbandDeg) || config.deadbandDeg < 0 || !finite(config.faultDeg) || config.faultDeg <= 0) {
            throw new IllegalArgumentException("kP, maxCorrection (0..1), deadband and faultDeg must be finite and in range");
        }
        this.c = config;
    }

    /** Heading to hold. Call once at the start of the run, after the IMU yaw is reset. */
    public void setTarget(double headingDeg) {
        if (!finite(headingDeg)) return;
        target = wrap(headingDeg);
        targetSet = true;
    }

    public boolean faulted() { return faulted; }
    public String faultReason() { return faultReason; }
    public double lastErrorDeg() { return lastError; }

    /** Yaw command to add to the drive command, positive counterclockwise. Zero if not usable. */
    public double correction(double headingDeg) {
        if (faulted || !targetSet) return 0;
        if (!finite(headingDeg)) {
            if (++badReadings >= c.maxBadReadings) fault("IMU heading unreadable");
            return 0;
        }
        badReadings = 0;
        double error = wrap(target - headingDeg); // positive: robot must turn counterclockwise
        lastError = error;
        if (Math.abs(error) > c.faultDeg) {
            fault(String.format("heading error %.0f deg exceeds %.0f deg (IMU orientation wrong, sign wrong, or the robot was hit)",
                    Math.abs(error), c.faultDeg));
            return 0;
        }
        if (Math.abs(error) <= c.deadbandDeg) return 0;
        double out = c.kP * error * (c.reversed ? -1.0 : 1.0);
        return Math.max(-c.maxCorrection, Math.min(c.maxCorrection, out));
    }

    private void fault(String reason) {
        faulted = true;
        faultReason = reason;
    }

    /** Wraps to the range (-180, 180]. */
    public static double wrap(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d <= -180.0) d += 360.0;
        return d;
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
