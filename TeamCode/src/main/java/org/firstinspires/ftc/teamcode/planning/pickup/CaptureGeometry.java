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
        return along(piece, heading, leadInches);
    }

    /**
     * Preserve the ordinary approach when feasible. Otherwise test eight fixed
     * compass directions, starting at +X and proceeding CCW. This includes
     * angled, wall-parallel, and inward approaches without general path search.
     * Null means no tested approach is feasible; never clamp an illegal endpoint.
     */
    public static PickupTarget feasibleThrough(TrackedPiece piece, double fromX, double fromY,
            double fromHeadingRad, double leadInches, CaptureFeasibilityModel feasibility) {
        if (piece == null || feasibility == null) return null;
        PickupTarget ordinary = through(piece, fromX, fromY, fromHeadingRad, leadInches);
        if (feasibility.isFeasible(fromX, fromY, fromHeadingRad, ordinary)) return ordinary;
        for (int direction = 0; direction < 8; direction++) {
            PickupTarget alternative = along(piece, direction * Math.PI / 4.0, leadInches);
            if (feasibility.isFeasible(fromX, fromY, fromHeadingRad, alternative)) return alternative;
        }
        return null;
    }

    private static PickupTarget along(TrackedPiece piece, double heading, double leadInches) {
        double lead = Double.isFinite(leadInches) && leadInches > 0.0 ? leadInches : 0.0;
        double captureX = piece.getFieldX() + (Math.cos(heading) * lead);
        double captureY = piece.getFieldY() + (Math.sin(heading) * lead);
        return new PickupTarget(piece, captureX, captureY, heading);
    }
}
