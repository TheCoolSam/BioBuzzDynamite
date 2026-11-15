package org.firstinspires.ftc.teamcode.turret;

import org.firstinspires.ftc.teamcode.vision.pieces.CellTargetGeometry;
import org.firstinspires.ftc.teamcode.vision.pieces.LimelightCameraPose;

/**
 * Aim at a point defined in the AprilTag's own frame (for example "4 inches above the tag, toward
 * the opening") instead of at the tag centre.
 *
 * <p>The offset is rotated by the tag's measured orientation, so it stays attached to the tag as the
 * tag moves. That matters on BioBuzz because the tags ride on the CELLS that tip.
 *
 * <p>Same math and convention as {@code CellTargetGeometry.offset}: camera space is X right, Y down,
 * Z forward; the tag-to-camera rotation comes from {@link LimelightCameraPose}. The sign of each
 * tag-frame axis is NOT assumed here. Verify it on the robot: change one offset axis and watch which
 * way {@link Result#elevationDeg} or {@link Result#yawDeg} moves.
 *
 * <p>Pure math, no SDK. Units: inches and degrees.
 */
public final class AimPoint {
    private AimPoint() { }

    public static final class Result {
        public boolean valid;
        /** Bearing to the point, positive = right of the camera axis (same sign as Limelight tx). */
        public double yawDeg = Double.NaN;
        /** Angle above the camera axis, positive = up (same sign as Limelight ty). */
        public double elevationDeg = Double.NaN;
        public double horizontalRangeIn = Double.NaN;
        public double slantRangeIn = Double.NaN;
        /** Same quantities for the unshifted tag centre, for comparison and sanity checks. */
        public double tagYawDeg = Double.NaN;
        public double tagElevationDeg = Double.NaN;
        public double tagHorizontalRangeIn = Double.NaN;
        public String reason = "";
    }

    /**
     * @param pose6 Limelight {@code [x, y, z, pitchDeg, yawDeg, rollDeg]} target pose in camera space
     * @param offX  offset along tag-frame X, inches
     * @param offY  offset along tag-frame Y, inches
     * @param offZ  offset along tag-frame Z, inches
     */
    public static Result compute(double[] pose6, double offX, double offY, double offZ) {
        Result r = new Result();
        if (pose6 == null || pose6.length < 6) { r.reason = "NO POSE"; return r; }
        for (int i = 0; i < 6; i++) if (!finite(pose6[i])) { r.reason = "POSE NOT FINITE"; return r; }
        if (!finite(offX) || !finite(offY) || !finite(offZ)) { r.reason = "OFFSET NOT FINITE"; return r; }
        if (pose6[2] <= 0) { r.reason = "TAG BEHIND CAMERA"; return r; }

        double[] rot = LimelightCameraPose.rotationFromTargetPoseCameraSpace(pose6);
        if (rot == null) { r.reason = "NO ROTATION"; return r; }

        double px = pose6[0] + rot[0] * offX + rot[1] * offY + rot[2] * offZ;
        double py = pose6[1] + rot[3] * offX + rot[4] * offY + rot[5] * offZ;
        double pz = pose6[2] + rot[6] * offX + rot[7] * offY + rot[8] * offZ;
        if (pz <= 0) { r.reason = "AIM POINT BEHIND CAMERA"; return r; }

        r.yawDeg = Math.toDegrees(Math.atan2(px, pz));
        r.horizontalRangeIn = Math.hypot(px, pz);
        r.elevationDeg = Math.toDegrees(Math.atan2(-py, r.horizontalRangeIn));
        r.slantRangeIn = Math.sqrt(px * px + py * py + pz * pz);

        r.tagYawDeg = Math.toDegrees(Math.atan2(pose6[0], pose6[2]));
        r.tagHorizontalRangeIn = Math.hypot(pose6[0], pose6[2]);
        r.tagElevationDeg = Math.toDegrees(Math.atan2(-pose6[1], r.tagHorizontalRangeIn));
        r.valid = true;
        return r;
    }

    /** Exposed so tests and tools can build the same tag-to-camera rotation. */
    public static double[] rotation(double yawDeg, double pitchDeg, double rollDeg) {
        return CellTargetGeometry.rotation(Math.toRadians(yawDeg), Math.toRadians(pitchDeg), Math.toRadians(rollDeg));
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
