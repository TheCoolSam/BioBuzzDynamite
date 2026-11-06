package org.firstinspires.ftc.teamcode.state;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import java.util.ArrayList;
import java.util.List;

/**
 * Static cross-OpMode persistence store.
 * Allows Autonomous to hand off estimated robot pose and indexed inventory
 * to TeleOp without requiring manual field re-zeroing.
 */
public final class PoseStorage {
    private static Pose currentPose = null;
    private static final List<BallType> carriedInventory = new ArrayList<BallType>();
    private static boolean poseValid = false;

    private PoseStorage() {}

    /**
     * Store the ending autonomous pose and remaining inventory.
     */
    public static synchronized void save(Pose pose, List<BallType> inventory) {
        if (pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading())) {
            currentPose = new Pose(pose.x(), pose.y(), pose.heading());
            poseValid = true;
        }
        carriedInventory.clear();
        if (inventory != null) {
            carriedInventory.addAll(inventory);
        }
    }

    /**
     * Store pose directly using coordinates and heading.
     */
    public static synchronized void save(double x, double y, double heading, List<BallType> inventory) {
        if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)) {
            currentPose = new Pose(x, y, heading);
            poseValid = true;
        }
        carriedInventory.clear();
        if (inventory != null) {
            carriedInventory.addAll(inventory);
        }
    }

    /**
     * Check if a valid handoff pose is available.
     */
    public static synchronized boolean hasPose() {
        return poseValid && currentPose != null;
    }

    /**
     * Retrieve the stored pose.
     */
    public static synchronized Pose getPose() {
        return currentPose != null ? new Pose(currentPose.x(), currentPose.y(), currentPose.heading()) : null;
    }

    /**
     * Retrieve a copy of the stored inventory.
     */
    public static synchronized List<BallType> getInventory() {
        return new ArrayList<BallType>(carriedInventory);
    }

    /**
     * Reset the stored state (e.g. at the start of Autonomous or upon manual reset).
     */
    public static synchronized void clear() {
        currentPose = null;
        carriedInventory.clear();
        poseValid = false;
    }
}
