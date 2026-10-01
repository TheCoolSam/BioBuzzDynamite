package org.firstinspires.ftc.teamcode.turret;

import org.firstinspires.ftc.teamcode.state.RobotState;

/**
 * Copies field motion from a {@link RobotState} into a {@link TurretState}.
 * The turret controller still does not know about robot snapshots.
 *
 * <p>Distances stay in inches, the same unit {@link RobotState} uses. The turret
 * target must be in inches too. An invalid pose is copied through with
 * {@link TurretState#poseValid} false. It is not replaced with the origin.
 * The controller decides to hold. Invalid velocity is also copied, and the
 * controller drops feedforward while it keeps aiming.
 * Nonfinite turret measurements, target coordinates, and loop times are
 * preserved so the controller can refuse tracking and command zero power.
 */
public final class TurretStateAdapter {

    private TurretStateAdapter() {
    }

    public static TurretState toTurretState(
            RobotState robot,
            double turretAngleRad,
            double turretVelocityRadPerSec,
            double targetXInches,
            double targetYInches,
            double dtSec) {
        TurretState turretState = new TurretState();
        if (robot == null) {
            turretState.poseValid = false;
            turretState.velocityValid = false;
        } else {
            turretState.robotX = robot.getFieldX();
            turretState.robotY = robot.getFieldY();
            turretState.robotHeading = robot.getHeadingRad();
            turretState.robotVx = robot.getFieldVx();
            turretState.robotVy = robot.getFieldVy();
            turretState.robotAngularVelocity = robot.getAngularVelocityRadPerSec();
            turretState.poseValid = robot.isPoseValid();
            turretState.velocityValid = robot.isVelocityValid();
        }
        // Preserve invalid measurements so the controller's input check can
        // refuse them. A fabricated zero angle could command a failed encoder
        // toward a target at full power.
        turretState.turretAngle = turretAngleRad;
        turretState.turretVelocity = turretVelocityRadPerSec;
        turretState.targetX = targetXInches;
        turretState.targetY = targetYInches;
        turretState.dt = dtSec;
        return turretState;
    }
}
