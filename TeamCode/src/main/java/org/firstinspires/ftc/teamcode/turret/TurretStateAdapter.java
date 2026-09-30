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
        turretState.turretAngle = finiteOrZero(turretAngleRad);
        turretState.turretVelocity = finiteOrZero(turretVelocityRadPerSec);
        turretState.targetX = finiteOrZero(targetXInches);
        turretState.targetY = finiteOrZero(targetYInches);
        turretState.dt = finiteOrZero(dtSec);
        return turretState;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
