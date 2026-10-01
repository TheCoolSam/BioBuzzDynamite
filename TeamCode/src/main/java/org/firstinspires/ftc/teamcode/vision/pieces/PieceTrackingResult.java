package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * What the tracker produced for one camera read.
 *
 * <p>{@link #getPieces()} is the only list the pickup planner should see.
 * It is empty when the floor calibration is missing or no observation has a
 * pose-valid historical robot state. Sightings can still describe the current
 * image, and robot-floor inches when the calibration exists, without becoming
 * planner pieces. Historical coordinates are meaningful only when
 * {@link Sighting#hasHistoricalPose()} is true.
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
        private final double readTimestampSec;
        private final double captureTimestampSec;
        private final boolean historicalPoseFound;
        private final double historicalX;
        private final double historicalY;
        private final double historicalHeadingRad;

        Sighting(
                int cameraId,
                BallType type,
                double imageX,
                double imageY,
                Double robotX,
                Double robotY,
                double readTimestampSec,
                double captureTimestampSec,
                RobotState historicalPose) {
            this.cameraId = cameraId;
            this.type = type;
            this.imageX = imageX;
            this.imageY = imageY;
            this.robotX = robotX;
            this.robotY = robotY;
            this.readTimestampSec = readTimestampSec;
            this.captureTimestampSec = captureTimestampSec;
            this.historicalPoseFound = historicalPose != null && historicalPose.isPoseValid();
            if (this.historicalPoseFound) {
                this.historicalX = historicalPose.getFieldX();
                this.historicalY = historicalPose.getFieldY();
                this.historicalHeadingRad = historicalPose.getHeadingRad();
            } else {
                this.historicalX = 0.0;
                this.historicalY = 0.0;
                this.historicalHeadingRad = 0.0;
            }
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

        public double getReadTimestampSec() {
            return readTimestampSec;
        }

        public double getCaptureTimestampSec() {
            return captureTimestampSec;
        }

        public boolean hasHistoricalPose() {
            return historicalPoseFound;
        }

        public double getHistoricalX() {
            return historicalX;
        }

        public double getHistoricalY() {
            return historicalY;
        }

        public double getHistoricalHeadingRad() {
            return historicalHeadingRad;
        }
    }
}
