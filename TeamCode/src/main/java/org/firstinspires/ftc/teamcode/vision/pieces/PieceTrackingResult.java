package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * What the tracker produced for one camera read.
 *
 * <p>{@link #getPieces()} is the only list the pickup planner should see.
 * It is empty when the floor calibration is missing or the robot pose is
 * invalid. Sightings can still describe the current image, and robot-floor
 * inches when the calibration exists, without becoming planner pieces.
 */
public final class PieceTrackingResult {

    private final List<TrackedPiece> pieces;
    private final List<Sighting> sightings;
    private final boolean calibrationConfigured;
    private final boolean poseUsed;
    private final int rawCount;
    private final int ignoredUnknownIds;

    PieceTrackingResult(
            List<TrackedPiece> pieces,
            List<Sighting> sightings,
            boolean calibrationConfigured,
            boolean poseUsed,
            int rawCount,
            int ignoredUnknownIds) {
        this.pieces = Collections.unmodifiableList(pieces);
        this.sightings = Collections.unmodifiableList(sightings);
        this.calibrationConfigured = calibrationConfigured;
        this.poseUsed = poseUsed;
        this.rawCount = rawCount;
        this.ignoredUnknownIds = ignoredUnknownIds;
    }

    public List<TrackedPiece> getPieces() {
        return pieces;
    }

    public List<Sighting> getSightings() {
        return sightings;
    }

    public boolean isCalibrationConfigured() {
        return calibrationConfigured;
    }

    public boolean isPoseUsed() {
        return poseUsed;
    }

    public int getRawCount() {
        return rawCount;
    }

    public int getIgnoredUnknownIds() {
        return ignoredUnknownIds;
    }

    /** One current detection after optional floor projection. Not a track. */
    public static final class Sighting {
        private final int cameraId;
        private final BallType type;
        private final double imageX;
        private final double imageY;
        private final Double robotX;
        private final Double robotY;

        Sighting(int cameraId, BallType type, double imageX, double imageY, Double robotX, Double robotY) {
            this.cameraId = cameraId;
            this.type = type;
            this.imageX = imageX;
            this.imageY = imageY;
            this.robotX = robotX;
            this.robotY = robotY;
        }

        public int getCameraId() {
            return cameraId;
        }

        public BallType getType() {
            return type;
        }

        public double getImageX() {
            return imageX;
        }

        public double getImageY() {
            return imageY;
        }

        public boolean hasRobotPoint() {
            return robotX != null && robotY != null;
        }

        public double getRobotX() {
            return robotX == null ? 0.0 : robotX;
        }

        public double getRobotY() {
            return robotY == null ? 0.0 : robotY;
        }
    }
}
