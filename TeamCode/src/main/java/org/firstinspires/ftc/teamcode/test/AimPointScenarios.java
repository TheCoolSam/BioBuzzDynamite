package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.turret.AimPoint;

/** Desktop checks for tag-frame aim points. */
public final class AimPointScenarios {
    private static int checks;

    public static void main(String[] args) {
        // Tag dead ahead at 60 in, facing the camera: zero offset aims at the tag centre.
        AimPoint.Result r = AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, 0, 0, 0);
        check(r.valid, "valid pose");
        near(r.yawDeg, 0, 1e-9, "no yaw to a centred tag");
        near(r.elevationDeg, 0, 1e-9, "no elevation to a centred tag");
        near(r.horizontalRangeIn, 60, 1e-9, "range is camera depth");
        check(r.yawDeg == r.tagYawDeg && r.elevationDeg == r.tagElevationDeg, "zero offset equals tag centre");

        // Tag to the right: positive yaw, matching Limelight tx sign.
        r = AimPoint.compute(new double[]{6, 0, 60, 0, 0, 0}, 0, 0, 0);
        near(r.yawDeg, Math.toDegrees(Math.atan(0.1)), 1e-9, "tag right of axis gives positive yaw");

        // Tag above the axis (camera Y is down, so y < 0): positive elevation, matching Limelight ty sign.
        r = AimPoint.compute(new double[]{0, -6, 60, 0, 0, 0}, 0, 0, 0);
        near(r.elevationDeg, Math.toDegrees(Math.atan(0.1)), 1e-9, "tag above axis gives positive elevation");

        // Facing the camera, tag-frame Y maps to camera Y (down). +4 in on tag Y lowers the aim point.
        AimPoint.Result down = AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, 0, 4, 0);
        AimPoint.Result up = AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, 0, -4, 0);
        check(down.elevationDeg < 0 && up.elevationDeg > 0, "tag-frame Y offset moves the aim point vertically");
        near(up.elevationDeg, Math.toDegrees(Math.atan2(4, 60)), 1e-9, "elevation of a 4 in rise at 60 in");
        near(up.tagElevationDeg, 0, 1e-9, "tag-centre comparison stays unshifted");

        // Tag-frame X offset shifts yaw; the shifted yaw differs from the tag yaw by the expected angle.
        r = AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, 3, 0, 0);
        near(r.yawDeg, Math.toDegrees(Math.atan2(3, 60)), 1e-9, "tag-frame X offset shifts yaw");

        // Same inches are fewer degrees farther away (why a fixed degree trim is wrong at other ranges).
        AimPoint.Result near30 = AimPoint.compute(new double[]{0, 0, 30, 0, 0, 0}, 3, 0, 0);
        AimPoint.Result far90 = AimPoint.compute(new double[]{0, 0, 90, 0, 0, 0}, 3, 0, 0);
        check(near30.yawDeg > 2 * far90.yawDeg, "fixed inches shrink in degrees with range");

        // Rotated tag: the offset rotates with the tag. Compare against the same rotation applied by hand.
        double[] pose = {5, -2, 70, 10, 35, -8};
        double[] rot = AimPoint.rotation(35, 10, -8);
        double ox = 2, oy = -3, oz = 1.5;
        double px = pose[0] + rot[0] * ox + rot[1] * oy + rot[2] * oz;
        double py = pose[1] + rot[3] * ox + rot[4] * oy + rot[5] * oz;
        double pz = pose[2] + rot[6] * ox + rot[7] * oy + rot[8] * oz;
        r = AimPoint.compute(pose, ox, oy, oz);
        near(r.yawDeg, Math.toDegrees(Math.atan2(px, pz)), 1e-9, "rotated tag yaw uses the rotated offset");
        near(r.elevationDeg, Math.toDegrees(Math.atan2(-py, Math.hypot(px, pz))), 1e-9, "rotated tag elevation");
        double dyaw = Math.abs(r.yawDeg - r.tagYawDeg) + Math.abs(r.elevationDeg - r.tagElevationDeg);
        check(dyaw > 0.5, "a rotated tag with an offset really moves the aim point");

        // Yaw of the tag changes where the same tag-frame offset ends up.
        AimPoint.Result facing = AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, 4, 0, 0);
        AimPoint.Result turned = AimPoint.compute(new double[]{0, 0, 60, 0, 60, 0}, 4, 0, 0);
        check(Math.abs(facing.yawDeg - turned.yawDeg) > 1.0, "tag orientation changes the aim point");

        // Bad inputs fail closed.
        check(!AimPoint.compute(null, 0, 0, 0).valid, "null pose rejected");
        check(!AimPoint.compute(new double[]{0, 0, 60, 0, 0}, 0, 0, 0).valid, "short pose rejected");
        check(!AimPoint.compute(new double[]{Double.NaN, 0, 60, 0, 0, 0}, 0, 0, 0).valid, "NaN pose rejected");
        check(!AimPoint.compute(new double[]{0, 0, -5, 0, 0, 0}, 0, 0, 0).valid, "tag behind camera rejected");
        check(!AimPoint.compute(new double[]{0, 0, 60, 0, 0, 0}, Double.NaN, 0, 0).valid, "NaN offset rejected");
        check(!AimPoint.compute(new double[]{0, 0, 5, 0, 0, 0}, 0, 0, -20).valid, "aim point behind camera rejected");

        System.out.println("AIM POINT CHECKS PASSED (" + checks + ")");
    }

    private static void near(double actual, double expected, double tol, String label) {
        checks++;
        if (!(Math.abs(actual - expected) <= tol)) throw new AssertionError(label + ": " + actual + " vs " + expected);
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }
}
