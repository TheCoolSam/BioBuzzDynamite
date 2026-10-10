package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Time from the route endpoint to the nearest feasible scoring pose, plus the
 * injected full load cycle. Mecanum can translate and yaw together, so the move is
 * the slower of those two, not their sum:
 * {@code max(translation, rotation) + shot}.
 *
 * <p>The fixed-delay constructors support simulations. Hardware uses the
 * measured cycle/footprint constructor; unknown load times reject that option.
 */
public final class PreferredPoseShotSetup implements ShotSetupTimeModel {

    /** An injected scoring pose. Inches and radians, project frame. */
    public static final class ShotPose {
        private final double x;
        private final double y;
        private final double headingRad;

        public ShotPose(double x, double y, double headingRad) {
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeadingRad() {
            return headingRad;
        }
    }

    private final List<ShotPose> poses;
    private final double speedInPerSec;
    private final double yawRateRadPerSec;
    private final double shotExecutionSec;
    private CycleTimeModel cycle;
    private SweptCaptureFeasibility footprint;
    public interface CycleTimeModel { double seconds(BallLoad load); }

    public PreferredPoseShotSetup(ShotPose... poses) {
        this(
                poses,
                PickupPlannerConstants.NOMINAL_SPEED_IN_PER_SEC,
                PickupPlannerConstants.YAW_RATE_RAD_PER_SEC,
                PickupPlannerConstants.SHOT_EXECUTION_SEC);
    }

    public PreferredPoseShotSetup(
            ShotPose[] poses,
            double speedInPerSec,
            double yawRateRadPerSec,
            double shotExecutionSec) {
        List<ShotPose> copy = new ArrayList<ShotPose>();
        if (poses != null) {
            for (int i = 0; i < poses.length; i++) {
                if (poses[i] != null) {
                    copy.add(poses[i]);
                }
            }
        }
        this.poses = Collections.unmodifiableList(copy);
        this.speedInPerSec = Double.isFinite(speedInPerSec)&&speedInPerSec>0 ? speedInPerSec : Double.NaN;
        this.yawRateRadPerSec = Double.isFinite(yawRateRadPerSec)&&yawRateRadPerSec>0 ? yawRateRadPerSec : Double.NaN;
        this.shotExecutionSec = Double.isFinite(shotExecutionSec) && shotExecutionSec >= 0.0
                ? shotExecutionSec
                : Double.NaN;
    }

    /** Hardware model prices the complete measured load cycle and requires a clear scoring corridor. */
    public PreferredPoseShotSetup(ShotPose[] poses,double speed,double yawRate,CycleTimeModel cycle,
            SweptCaptureFeasibility footprint) {
        this(poses,speed,yawRate,0);
        if(cycle==null||footprint==null)throw new IllegalArgumentException("Measured cycle and footprint required");
        this.cycle=cycle;this.footprint=footprint;
    }
    public boolean hasMeasuredCycle() {return cycle!=null&&footprint!=null;}

    @Override
    public double estimateSeconds(double robotX, double robotY, double robotHeading, BallLoad load) {
        ShotSetupPlan plan = select(robotX, robotY, robotHeading, load);
        return plan == null ? Double.NaN : plan.seconds;
    }

    public ShotSetupPlan select(double robotX, double robotY, double robotHeading, BallLoad load) {
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY) || !Double.isFinite(robotHeading)) {
            return null;
        }
        if (poses.isEmpty()) {
            return null;
        }
        double execution=cycle==null?shotExecutionSec:cycle.seconds(load);
        if(!Double.isFinite(execution)||execution<0||!Double.isFinite(speedInPerSec)||!Double.isFinite(yawRateRadPerSec))return null;
        double best = Double.POSITIVE_INFINITY;
        ShotPose chosen = null;
        for (int i = 0; i < poses.size(); i++) {
            ShotPose pose = poses.get(i);
            if (!Double.isFinite(pose.x) || !Double.isFinite(pose.y) || !Double.isFinite(pose.headingRad)) {
                continue;
            }
            if(footprint!=null&&!footprint.segmentSafe(robotX,robotY,pose.x,pose.y))continue;
            double distance = Math.hypot(pose.x - robotX, pose.y - robotY);
            double translation = distance / speedInPerSec;
            double yaw = Math.abs(AngleUtil.wrapRadians(pose.headingRad - robotHeading)) / yawRateRadPerSec;
            double seconds = Math.max(translation, yaw) + execution;
            if (seconds < best) {
                best = seconds;
                chosen = pose;
            }
        }
        return chosen == null ? null : new ShotSetupPlan(chosen.x, chosen.y, chosen.headingRad, best);
    }
}
