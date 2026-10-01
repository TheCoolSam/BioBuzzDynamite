package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Time from the route endpoint to the nearest injected scoring pose, plus a
 * fixed shot delay. Mecanum can translate and yaw together, so the move is
 * the slower of those two, not their sum:
 * {@code max(translation, rotation) + shot}.
 *
 * <p>The poses are placeholders supplied by the caller. This class does not
 * know where the HIVE is. {@link BallLoad} is accepted and ignored so a later
 * model can take longer for a heavier load without a planner change.
 */
public final class PreferredPoseShotSetup implements ShotSetupTimeModel {

    /** One fake place the robot can shoot from. Inches and radians, project frame. */
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
        this.speedInPerSec = speedInPerSec > 1.0e-3 ? speedInPerSec : 1.0e-3;
        this.yawRateRadPerSec = yawRateRadPerSec > 1.0e-3 ? yawRateRadPerSec : 1.0e-3;
        this.shotExecutionSec = Double.isFinite(shotExecutionSec) && shotExecutionSec > 0.0
                ? shotExecutionSec
                : 0.0;
    }

    @Override
    public double estimateSeconds(double robotX, double robotY, double robotHeading, BallLoad load) {
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY) || !Double.isFinite(robotHeading)) {
            return 1.0e6;
        }
        if (poses.isEmpty()) {
            return shotExecutionSec;
        }
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < poses.size(); i++) {
            ShotPose pose = poses.get(i);
            if (!Double.isFinite(pose.x) || !Double.isFinite(pose.y) || !Double.isFinite(pose.headingRad)) {
                continue;
            }
            double distance = Math.hypot(pose.x - robotX, pose.y - robotY);
            double translation = distance / speedInPerSec;
            double yaw = Math.abs(AngleUtil.wrapRadians(pose.headingRad - robotHeading)) / yawRateRadPerSec;
            double seconds = Math.max(translation, yaw) + shotExecutionSec;
            if (seconds < best) {
                best = seconds;
            }
        }
        return Double.isFinite(best) ? best : 1.0e6;
    }
}
