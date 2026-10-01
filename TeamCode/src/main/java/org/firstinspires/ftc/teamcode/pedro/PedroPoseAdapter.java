package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.planning.pickup.PickupTarget;

/**
 * Turns project coordinates into the current Pedro {@link Pose}.
 *
 * <p>{@link PickupTarget} does not import Pedro. A later autonomous can take
 * each capture pose from a {@code PickupPlan}, convert it here, and follow
 * {@link Paths#line(Pose, Pose)} or {@link Paths#through(Pose...)} in order.
 * After the last capture, the same conversion turns the plan endpoint into
 * the shot pose. This class only builds that pose or a straight path. It
 * does not follow it, and it does not call the planner.
 */
public final class PedroPoseAdapter {

    private PedroPoseAdapter() {
    }

    /** Inches and radians, project frame. Null when any value is non-finite. */
    public static Pose toPose(double xInches, double yInches, double headingRad) {
        if (!Double.isFinite(xInches) || !Double.isFinite(yInches) || !Double.isFinite(headingRad)) {
            return null;
        }
        return new Pose(xInches, yInches, headingRad);
    }

    /** Capture point and approach heading. Null when the target is unusable. */
    public static Pose fromPickupTarget(PickupTarget target) {
        if (target == null) {
            return null;
        }
        return toPose(target.getCaptureX(), target.getCaptureY(), target.getApproachHeadingRad());
    }

    /**
     * Straight path from a start pose to the capture pose. Null when either
     * pose is missing. The caller decides whether {@code Follower.follow}
     * should run.
     */
    public static Path straightToCapture(Pose start, PickupTarget target) {
        Pose capture = fromPickupTarget(target);
        if (start == null || capture == null) {
            return null;
        }
        if (!Double.isFinite(start.x()) || !Double.isFinite(start.y()) || !Double.isFinite(start.heading())) {
            return null;
        }
        return Paths.line(start, capture);
    }
}
