package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/**
 * Tracker knobs. Camera ids are whatever was taught on the HuskyLens.
 * They are not field geometry, and they are not scattered through the tracker.
 *
 * <p>The default ids are placeholders until the device is trained.
 * Alliance ownership is not inferred from those ids.
 */
public final class PieceTrackerConstants {

    /** Taught HuskyLens id for {@link BallType#POLLEN}. Placeholder. */
    public final int pollenCameraId;

    /** Taught HuskyLens id for {@link BallType#NECTAR}. Placeholder. */
    public final int nectarCameraId;

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
     * False until a later rule says an unrecognized owner is legal to collect.
     * The recognition mode does not know alliance color. Ownership on every
     * track is {@link org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership#NEUTRAL}.
     */
    public final boolean collectWhenOwnershipUnknown;

    public PieceTrackerConstants(
            int pollenCameraId,
            int nectarCameraId,
            double associationGateInches,
            double trackTimeoutSec,
            double positionBlend,
            double matureHitCount,
            ObservationAnchor anchor,
            boolean collectWhenOwnershipUnknown) {
        this.pollenCameraId = pollenCameraId;
        this.nectarCameraId = nectarCameraId;
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
        this.collectWhenOwnershipUnknown = collectWhenOwnershipUnknown;
    }

    /**
     * Ids 1 and 2 are not a trained model. Gate, timeout, and blend are
     * starting values for a small number of balls, not a fit to video.
     * Unknown ownership is not collectable.
     */
    public static PieceTrackerConstants defaults() {
        return new PieceTrackerConstants(
                1,
                2,
                6.0,
                0.40,
                0.65,
                5.0,
                ObservationAnchor.LOWER_CENTER,
                false);
    }

    /** Null when this id was not taught as pollen or nectar. */
    public BallType typeFor(int cameraId) {
        if (cameraId == pollenCameraId && pollenCameraId != nectarCameraId) {
            return BallType.POLLEN;
        }
        if (cameraId == nectarCameraId) {
            return BallType.NECTAR;
        }
        if (cameraId == pollenCameraId) {
            return BallType.POLLEN;
        }
        return null;
    }
}
