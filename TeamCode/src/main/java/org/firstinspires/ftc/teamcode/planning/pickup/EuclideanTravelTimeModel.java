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
        this.speedInPerSec = Double.isFinite(speedInPerSec)&&speedInPerSec>0 ? speedInPerSec : Double.NaN;
        this.yawRateRadPerSec = Double.isFinite(yawRateRadPerSec)&&yawRateRadPerSec>0 ? yawRateRadPerSec : Double.NaN;
        this.acquisitionOverheadSec = Double.isFinite(acquisitionOverheadSec)&&acquisitionOverheadSec>=0 ? acquisitionOverheadSec : Double.NaN;
    }

    @Override
    public double estimateSeconds(
            double startX,
            double startY,
            double startHeadingRad,
            PickupTarget target) {
        if (target == null || target.getPiece()==null
                || !Double.isFinite(startX)
                || !Double.isFinite(startY)
                || !Double.isFinite(startHeadingRad)
                || !Double.isFinite(target.getCaptureX())
                || !Double.isFinite(target.getCaptureY())
                || !Double.isFinite(target.getApproachHeadingRad())) {
            return Double.NaN;
        }

        // Execute the staging segment before crossing the piece along the capture heading.
        double translation = (Math.hypot(target.getApproachX()-startX,target.getApproachY()-startY)
                +Math.hypot(target.getCaptureX()-target.getApproachX(),target.getCaptureY()-target.getApproachY())) / speedInPerSec;
        double yaw = Math.abs(AngleUtil.wrapRadians(
                target.getApproachHeadingRad() - startHeadingRad)) / yawRateRadPerSec;
        return Math.max(translation, yaw) + acquisitionOverheadSec;
    }
}
