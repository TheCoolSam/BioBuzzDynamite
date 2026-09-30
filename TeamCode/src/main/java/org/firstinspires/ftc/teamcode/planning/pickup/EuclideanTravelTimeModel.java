package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/**
 * Straight-line planning time. Distance over a nominal speed, plus the time
 * to yaw onto the approach, plus a fixed intake overhead.
 *
 * <p>This is not a path. It ignores walls, traffic, and mechanism limits.
 * Swap in another {@link TravelTimeModel} when a real follower exists.
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
        return translation + yaw + acquisitionOverheadSec;
    }
}
