package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * One game piece the planner is allowed to know about.
 *
 * <p>Field position is inches in the project frame (+X right, +Y forward).
 * This class is the seam for vision. A future tracker turns camera blocks
 * into these objects. Nothing here knows what a HuskyLens is.
 */
public final class TrackedPiece {

    private final int id;
    private final BallType type;
    private final PieceOwnership ownership;
    private final double fieldX;
    private final double fieldY;
    private final double confidence;
    private final double lastSeenTimestampSec;
    private final boolean collectable;

    public TrackedPiece(
            int id,
            BallType type,
            PieceOwnership ownership,
            double fieldX,
            double fieldY,
            double confidence,
            double lastSeenTimestampSec,
            boolean collectable) {
        this.id = id;
        this.type = type == null ? BallType.POLLEN : type;
        this.ownership = ownership == null ? PieceOwnership.NEUTRAL : ownership;
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.confidence = confidence;
        this.lastSeenTimestampSec = lastSeenTimestampSec;
        this.collectable = collectable;
    }

    public int getId() {
        return id;
    }

    public BallType getType() {
        return type;
    }

    public PieceOwnership getOwnership() {
        return ownership;
    }

    /** Inches. */
    public double getFieldX() {
        return fieldX;
    }

    /** Inches. */
    public double getFieldY() {
        return fieldY;
    }

    /** Detection confidence in [0, 1]. The planner clamps anything outside that. */
    public double getConfidence() {
        return confidence;
    }

    public double getLastSeenTimestampSec() {
        return lastSeenTimestampSec;
    }

    /**
     * False means the piece is not a legal or currently available target.
     * The planner does not inspect alliance color itself.
     */
    public boolean isCollectable() {
        return collectable;
    }
}
