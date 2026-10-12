package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * Limelight {@code targetpose_cameraspace} / fiducial {@code t6t_cs} is
 * {@code [tx, ty, tz, pitch, yaw, roll]} in camera space (X right, Y down,
 * Z forward), translations in the caller's units and angles in degrees.
 *
 * <p>FTC SDK 12 {@code Pose3D} helpers construct
 * {@code YawPitchRollAngles(yaw=a[5], pitch=a[4], roll=a[3])}, treating the
 * last three as roll/pitch/yaw. {@code getYaw/getPitch/getRoll} are therefore
 * Limelight roll/yaw/pitch. Feeding those labels into
 * {@code Rz(yaw)·Ry(pitch)·Rx(roll)} rotates a tag-to-CELL offset incorrectly
 * off-axis.
 *
 * <p>This class reads the documented array order and builds
 * {@code Rz(yaw)·Ry(pitch)·Rx(roll)} from Limelight yaw/pitch/roll.
 */
public final class LimelightCameraPose {
    private LimelightCameraPose() { }

    /** {@code pose6} is {@code [tx, ty, tz, pitchDeg, yawDeg, rollDeg]}. */
    public static double[] rotationFromTargetPoseCameraSpace(double[] pose6) {
        if (pose6 == null || pose6.length < 6) return null;
        for (int i = 3; i < 6; i++) if (!Double.isFinite(pose6[i])) return null;
        return CellTargetGeometry.rotation(
                Math.toRadians(pose6[4]),
                Math.toRadians(pose6[3]),
                Math.toRadians(pose6[5]));
    }

    /**
     * Recover the Limelight array from SDK-labeled angles, which store
     * {@code yaw=a[5]=roll}, {@code pitch=a[4]=yaw}, {@code roll=a[3]=pitch}.
     */
    public static double[] rotationFromSdkLabeledAnglesRad(double sdkYawRad, double sdkPitchRad, double sdkRollRad) {
        if (!Double.isFinite(sdkYawRad) || !Double.isFinite(sdkPitchRad) || !Double.isFinite(sdkRollRad)) {
            return null;
        }
        return CellTargetGeometry.rotation(sdkPitchRad, sdkRollRad, sdkYawRad);
    }
}
