package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/**
 * Builds the pose that drives the intake through a piece.
 *
 * <pre>
 *   robot --------&gt; piece --------&gt; capture pose
 * </pre>
 *
 * The capture point sits {@code leadInches} past the piece along the line
 * from the previous pose to the piece. If the robot is already on the piece,
 * the previous heading is the approach direction.
 */
public final class CaptureGeometry {

    private CaptureGeometry() {
    }

    public static PickupTarget through(
            TrackedPiece piece,
            double fromX,
            double fromY,
            double fromHeadingRad,
            double leadInches) {
        double heading = AngleUtil.fieldBearing(
                fromX,
                fromY,
                fromHeadingRad,
                piece.getFieldX(),
                piece.getFieldY());
        double lead = Double.isFinite(leadInches) && leadInches > 0.0 ? leadInches : 0.0;
        double captureX = piece.getFieldX() + (Math.cos(heading) * lead);
        double captureY = piece.getFieldY() + (Math.sin(heading) * lead);
        return new PickupTarget(piece, captureX, captureY, heading);
    }
}
