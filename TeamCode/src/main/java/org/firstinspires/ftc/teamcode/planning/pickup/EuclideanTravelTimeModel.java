package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/**
 * Straight-line planning time for a mecanum drivetrain. Translation and yaw
 * run together, so movement time is the longer of the two, then a fixed
 * intake overhead is added. This is the same overlap used by shot setup.
 * It is not a wheel-authority model and it is not a path.
 *
 * <p>Walls, traffic, and mechanism limits are ignored. A measured route or
 * Pedro can replace this model later.
 */
public final class EuclideanTravelTimeModel implements TravelTimeModel {

    private final double speedInPerSec;
    private final double yawRateRadPerSec;
    private final double acquisitionOverheadSec;

    public EuclideanTravelTimeModel() {
        this(
                PickupPlannerConstants.NOMINAL_SPEED_IN_PER_SEC,
                PickupPlannerConstants.YAW_RATE_RAD_PER_SEC,
                PickupPlannerConstants.ACQUISITION_OVERHEAD_SEC);
    }

    public EuclideanTravelTimeModel(
            double speedInPerSec,
            double yawRateRadPerSec,
            double acquisitionOverheadSec) {
        this.speedInPerSec = speedInPerSec > 1.0e-3 ? speedInPerSec : 1.0e-3;
        this.yawRateRadPerSec = yawRateRadPerSec > 1.0e-3 ? yawRateRadPerSec : 1.0e-3;
        this.acquisitionOverheadSec = acquisitionOverheadSec > 0.0 ? acquisitionOverheadSec : 0.0;
    }

    @Override
    public double estimateSeconds(
            double startX,
            double startY,
            double startHeadingRad,
            PickupTarget target) {
        if (target == null
                || !Double.isFinite(startX)
                || !Double.isFinite(startY)
                || !Double.isFinite(startHeadingRad)
                || !Double.isFinite(target.getCaptureX())
                || !Double.isFinite(target.getCaptureY())
                || !Double.isFinite(target.getApproachHeadingRad())) {
            return 1.0e6;
        }

        double dx = target.getCaptureX() - startX;
        double dy = target.getCaptureY() - startY;
        double translation = Math.hypot(dx, dy) / speedInPerSec;
        double yaw = Math.abs(AngleUtil.wrapRadians(
                target.getApproachHeadingRad() - startHeadingRad)) / yawRateRadPerSec;
        return Math.max(translation, yaw) + acquisitionOverheadSec;
    }
}
