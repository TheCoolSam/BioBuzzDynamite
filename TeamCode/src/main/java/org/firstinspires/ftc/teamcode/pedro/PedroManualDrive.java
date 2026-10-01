package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

/**
 * Field-centric drive request for later TeleOp. This is not the driver system.
 *
 * <p>{@code fieldX} is power along field +X (right). {@code fieldY} is power
 * along field +Y (forward). {@code chassisOmega} is turn power, positive
 * counterclockwise, the same sign as {@link org.firstinspires.ftc.teamcode.math.AngleUtil}.
 * Pedro's {@link ManualDrive#fieldCentric(double, double, double, double)} treats
 * its first two arguments as a world-frame vector and rotates that vector into
 * the robot: at heading 0, +X becomes forward and +Y becomes strafe-left,
 * which is +Y for a robot facing +X. Turn is passed through unchanged.
 *
 * <p>Does not call {@link Follower#update()} and does not read turret unwind.
 * A non-finite command or pose is refused and replaces the previous manual
 * demand with zero. The caller still runs its normal {@link Follower#update()}.
 */
public final class PedroManualDrive {

    private PedroManualDrive() {
    }

    /**
     * @return false when the command was refused
     */
    public static boolean request(
            Follower follower,
            double fieldX,
            double fieldY,
            double chassisOmega) {
        if (follower == null) {
            return false;
        }
        Pose pose = follower.pose();
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            follower.manual(DrivePowers.zero());
            return false;
        }
        DrivePowers powers = powers(fieldX, fieldY, chassisOmega, pose.heading());
        if (powers == null) {
            follower.manual(DrivePowers.zero());
            return false;
        }
        follower.manual(powers);
        return true;
    }

    /**
     * The {@link DrivePowers} Pedro would apply for this field command.
     * Null when any value is non-finite. Used by the no-hardware sign checks.
     */
    public static DrivePowers powers(
            double fieldX,
            double fieldY,
            double chassisOmega,
            double headingRad) {
        if (!Double.isFinite(fieldX)
                || !Double.isFinite(fieldY)
                || !Double.isFinite(chassisOmega)
                || !Double.isFinite(headingRad)) {
            return null;
        }
        return ManualDrive.fieldCentric(fieldX, fieldY, chassisOmega, headingRad);
    }
}
