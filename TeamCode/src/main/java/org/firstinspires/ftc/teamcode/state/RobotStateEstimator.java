package org.firstinspires.ftc.teamcode.state;

/**
 * Builds a fresh {@link RobotState} once per control loop.
 *
 * <p>Pose and velocity are copied from the {@link RobotStateSource}. Acceleration
 * is the change in velocity divided by dt. There is no smoothing yet.
 * Call {@link #update(RobotStateSource, double)} from the OpMode thread only.
 *
 * <p>A heading step from +179° to -179° is not treated as a spin of about -358°.
 * Heading on the snapshot is wrapped by {@link org.firstinspires.ftc.teamcode.math.AngleUtil}.
 * Angular acceleration comes from the source's angular velocity, which is already
 * a rate, so a wrapped heading jump cannot create a huge acceleration.
 *
 * <p>dt must be greater than {@link #MIN_DT_SEC}. A zero, negative, or non-finite
 * timestamp does not move the derivative baseline and produces zero acceleration.
 * The first valid velocity sample also produces zero acceleration. A sample with
 * invalid velocity clears that baseline so the next good sample does not difference
 * across the gap.
 */
public final class RobotStateEstimator {

    /** Smallest dt used for a finite difference. Shorter gaps report zero acceleration. */
    public static final double MIN_DT_SEC = 1.0e-4;

    private RobotState state = RobotState.invalid(0.0);
    private boolean hasBaseline = false;
    private double baselineTimeSec = 0.0;
    private double baselineVx = 0.0;
    private double baselineVy = 0.0;
    private double baselineOmega = 0.0;
    private long generation = Long.MIN_VALUE;

    public void reset() {
        hasBaseline = false;
        generation = Long.MIN_VALUE;
        baselineTimeSec = 0.0;
        baselineVx = 0.0;
        baselineVy = 0.0;
        baselineOmega = 0.0;
        state = RobotState.invalid(0.0);
    }

    public RobotState getState() {
        return state;
    }

    public RobotState update(RobotStateSource source, double timestampSec) {
        if (source == null || !Double.isFinite(timestampSec)) {
            hasBaseline = false;
            state = RobotState.invalid(Double.isFinite(timestampSec) ? timestampSec : 0.0);
            return state;
        }

        boolean poseFinite = finite(source.getX())
                && finite(source.getY())
                && finite(source.getHeadingRad());
        boolean velocityFinite = finite(source.getFieldVx())
                && finite(source.getFieldVy())
                && finite(source.getAngularVelocityRadPerSec());
        double acquired = source.getAcquisitionTimestampSec();
        boolean healthy = source.isDeviceHealthy() && finite(acquired) && acquired <= timestampSec + 1e-9;
        if (generation != source.getResetGeneration()) {
            hasBaseline = false;
            generation = source.getResetGeneration();
        }
        boolean poseValid = source.isPoseValid() && poseFinite && healthy;
        boolean velocityValid = source.isVelocityValid() && velocityFinite && healthy;

        double x = poseFinite ? source.getX() : 0.0;
        double y = poseFinite ? source.getY() : 0.0;
        double heading = poseFinite ? source.getHeadingRad() : 0.0;
        double vx = velocityFinite ? source.getFieldVx() : 0.0;
        double vy = velocityFinite ? source.getFieldVy() : 0.0;
        double omega = velocityFinite ? source.getAngularVelocityRadPerSec() : 0.0;

        double ax = 0.0;
        double ay = 0.0;
        double alpha = 0.0;
        double dt = acquired - baselineTimeSec;
        if (velocityValid && hasBaseline && dt > MIN_DT_SEC) {
            ax = (vx - baselineVx) / dt;
            ay = (vy - baselineVy) / dt;
            alpha = (omega - baselineOmega) / dt;
            if (!finite(ax) || !finite(ay) || !finite(alpha)) {
                ax = 0.0;
                ay = 0.0;
                alpha = 0.0;
            }
        }

        if (!velocityValid) {
            hasBaseline = false;
        } else if (!hasBaseline || acquired > baselineTimeSec) {
            hasBaseline = true;
            baselineTimeSec = acquired;
            baselineVx = vx;
            baselineVy = vy;
            baselineOmega = omega;
        }

        state = new RobotState(
                timestampSec,
                x,
                y,
                heading,
                vx,
                vy,
                omega,
                ax,
                ay,
                alpha,
                poseValid,
                velocityValid, acquired, healthy, generation);
        return state;
    }

    private static boolean finite(double value) {
        return Double.isFinite(value);
    }
}
