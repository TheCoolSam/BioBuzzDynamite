package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Where the drivetrain should go to collect one piece.
 *
 * <p>The capture point is past the piece, not on it. A later path follower
 * drives through the ball and finishes at this pose. Heading is the approach
 * direction in the project frame (0 along +X, counterclockwise positive).
 */
public final class PickupTarget {

    private final TrackedPiece piece;
    private final double captureX;
    private final double captureY;
    private final double approachHeadingRad;

    public PickupTarget(
            TrackedPiece piece,
            double captureX,
            double captureY,
            double approachHeadingRad) {
        this.piece = piece;
        this.captureX = captureX;
        this.captureY = captureY;
        this.approachHeadingRad = approachHeadingRad;
    }

    public TrackedPiece getPiece() {
        return piece;
    }

    /** Inches. */
    public double getCaptureX() {
        return captureX;
    }

    /** Inches. */
    public double getCaptureY() {
        return captureY;
    }

    public double getApproachHeadingRad() {
        return approachHeadingRad;
    }
}
