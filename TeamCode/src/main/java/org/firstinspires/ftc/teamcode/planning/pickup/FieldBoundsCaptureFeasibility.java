package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * V0 rectangular field gate. Both start and capture center must remain within
 * the rectangle inset by independently configured X/Y wall clearances. A straight
 * center segment between them also stays in this convex rectangle. Does not
 * certify an intake approach, a swept rotating body, or obstacle avoidance.
 * Clearance comes from the caller; this class invents no CAD footprint.
 */
public final class FieldBoundsCaptureFeasibility implements CaptureFeasibilityModel {
    private final double minX, minY, maxX, maxY;
    private final double centerMinX, centerMinY, centerMaxX, centerMaxY;

    public FieldBoundsCaptureFeasibility(double width, double height, double clearanceX, double clearanceY) {
        this(0.0, 0.0, width, height, clearanceX, clearanceY);
    }

    public FieldBoundsCaptureFeasibility(double originX, double originY, double width, double height,
            double clearanceX, double clearanceY) {
        if (!Double.isFinite(originX) || !Double.isFinite(originY)
                || !Double.isFinite(width) || !Double.isFinite(height)
                || !Double.isFinite(clearanceX) || !Double.isFinite(clearanceY)
                || width <= 0 || height <= 0 || clearanceX < 0 || clearanceY < 0
                || 2 * clearanceX > width || 2 * clearanceY > height
                || !Double.isFinite(originX + width) || !Double.isFinite(originY + height)) {
            throw new IllegalArgumentException("Finite field dimensions and legal wall clearances required");
        }
        minX = originX;
        minY = originY;
        maxX = originX + width;
        maxY = originY + height;
        centerMinX = minX + clearanceX;
        centerMinY = minY + clearanceY;
        centerMaxX = maxX - clearanceX;
        centerMaxY = maxY - clearanceY;
    }

    @Override
    public boolean isFeasible(double startX, double startY, double startHeadingRad, PickupTarget target) {
        if (target == null || target.getPiece() == null || !Double.isFinite(startHeadingRad)
                || !Double.isFinite(target.getApproachHeadingRad())) return false;
        TrackedPiece piece = target.getPiece();
        return insideCenter(startX, startY)
                && insideCenter(target.getCaptureX(), target.getCaptureY())
                && piece.getFieldX() >= minX && piece.getFieldX() <= maxX
                && piece.getFieldY() >= minY && piece.getFieldY() <= maxY;
    }

    private boolean insideCenter(double x, double y) {
        return x >= centerMinX && x <= centerMaxX && y >= centerMinY && y <= centerMaxY;
    }
}
