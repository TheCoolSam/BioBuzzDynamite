package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;

/**
 * Tracker knobs and the HuskyLens id map.
 *
 * <p>Ids 1, 2, and 3 are placeholders until pollen, red nectar, and blue
 * nectar are taught on the device. They are not scattered through the tracker.
 * {@link #HUSKYLENS_LATENCY_SEC} stays 0 until the camera delay is measured.
 * Alliance color is configuration for this layer. The pickup planner never sees it.
 */
public final class PieceTrackerConstants {

    /** Seconds subtracted from the read time. 0 until a measurement exists. Not a guess. */
    public static final double HUSKYLENS_LATENCY_SEC = 0.0;

    /** Taught HuskyLens id for {@link BallType#POLLEN}. Placeholder, not a trained model. */
    public final int pollenCameraId;

    /** Taught HuskyLens id for red nectar. Placeholder, not a trained model. */
    public final int redNectarCameraId;

    /** Taught HuskyLens id for blue nectar. Placeholder, not a trained model. */
    public final int blueNectarCameraId;

    public final AllianceColor alliance;

    /** Seconds. Subtracted from each observation read time before the pose lookup. */
    public final double huskyLensLatencySec;

    /** Inches. Observations farther than this from a track start a new track. */
    public final double associationGateInches;

    /** Seconds without a match before a track is dropped. */
    public final double trackTimeoutSec;

    /**
     * Blend toward the new observation. 0 keeps the old point, 1 snaps to
     * the new one. Balls can roll, so the stored point is allowed to move.
     */
    public final double positionBlend;

    /** Hits that bring detection confidence from a new track up to 1. */
    public final double matureHitCount;

    public final ObservationAnchor anchor;

    /**
     * Pollen is neutral, not an alliance piece. This flag is the collectability
     * policy for that known case. Unknown nectar does not use it.
     */
    public final boolean collectNeutralPollen;

    public PieceTrackerConstants(
            int pollenCameraId,
            int redNectarCameraId,
            int blueNectarCameraId,
            AllianceColor alliance,
            double huskyLensLatencySec,
            double associationGateInches,
            double trackTimeoutSec,
            double positionBlend,
            double matureHitCount,
            ObservationAnchor anchor,
            boolean collectNeutralPollen) {
        this.pollenCameraId = pollenCameraId;
        this.redNectarCameraId = redNectarCameraId;
        this.blueNectarCameraId = blueNectarCameraId;
        this.alliance = alliance == null ? AllianceColor.UNKNOWN : alliance;
        if (!Double.isFinite(huskyLensLatencySec) || huskyLensLatencySec < 0.0) {
            this.huskyLensLatencySec = 0.0;
        } else {
            this.huskyLensLatencySec = huskyLensLatencySec;
        }
        this.associationGateInches = associationGateInches > 0.0 ? associationGateInches : 0.0;
        this.trackTimeoutSec = trackTimeoutSec > 0.0 ? trackTimeoutSec : 0.0;
        if (!Double.isFinite(positionBlend)) {
            this.positionBlend = 1.0;
        } else if (positionBlend < 0.0) {
            this.positionBlend = 0.0;
        } else if (positionBlend > 1.0) {
            this.positionBlend = 1.0;
        } else {
            this.positionBlend = positionBlend;
        }
        this.matureHitCount = matureHitCount > 0.0 ? matureHitCount : 1.0;
        this.anchor = anchor == null ? ObservationAnchor.LOWER_CENTER : anchor;
        this.collectNeutralPollen = collectNeutralPollen;
    }

    /**
     * Placeholder ids. Alliance is unknown, so nectar is not collectable.
     * Latency is {@link #HUSKYLENS_LATENCY_SEC}.
     */
    public static PieceTrackerConstants defaults() {
        return new PieceTrackerConstants(
                1,
                2,
                3,
                AllianceColor.UNKNOWN,
                HUSKYLENS_LATENCY_SEC,
                6.0,
                0.40,
                0.65,
                5.0,
                ObservationAnchor.LOWER_CENTER,
                true);
    }

    /**
     * Maps one taught camera id. An id that matches more than one class, or
     * none, returns null and must be ignored.
     */
    public ClassifiedPiece classify(int cameraId) {
        boolean pollen = cameraId == pollenCameraId;
        boolean red = cameraId == redNectarCameraId;
        boolean blue = cameraId == blueNectarCameraId;
        int hits = (pollen ? 1 : 0) + (red ? 1 : 0) + (blue ? 1 : 0);
        if (hits != 1) {
            return null;
        }
        if (pollen) {
            return new ClassifiedPiece(BallType.POLLEN, PieceOwnership.NEUTRAL, collectNeutralPollen);
        }
        if (alliance == AllianceColor.UNKNOWN) {
            return new ClassifiedPiece(BallType.NECTAR, PieceOwnership.UNKNOWN, false);
        }
        boolean ours = (alliance == AllianceColor.RED && red) || (alliance == AllianceColor.BLUE && blue);
        if (ours) {
            return new ClassifiedPiece(BallType.NECTAR, PieceOwnership.ALLIANCE, true);
        }
        return new ClassifiedPiece(BallType.NECTAR, PieceOwnership.OPPONENT, false);
    }

    /** One camera class after alliance rules. Not a track. */
    public static final class ClassifiedPiece {
        private final BallType type;
        private final PieceOwnership ownership;
        private final boolean collectable;

        private ClassifiedPiece(BallType type, PieceOwnership ownership, boolean collectable) {
            this.type = type;
            this.ownership = ownership;
            this.collectable = collectable;
        }

        public BallType getType() {
            return type;
        }

        public PieceOwnership getOwnership() {
            return ownership;
        }

        public boolean isCollectable() {
            return collectable;
        }
    }
}
