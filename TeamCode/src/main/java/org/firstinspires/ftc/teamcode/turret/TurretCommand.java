package org.firstinspires.ftc.teamcode.turret;

/**
 * What the turret wants this cycle. The drive code is not in this task:
 * {@link #requestedChassisOmega} is only a request for a future mecanum controller.
 * Angles are radians. Motor power is in [-1, 1].
 */
public class TurretCommand {

    /** Field bearing from the robot to the target, radians. */
    public final double targetBearing;

    /**
     * Robot-relative angle the turret should hold.
     * Kept inside the operating window. The physical stops are only a final clamp.
     */
    public final double desiredTurretAngle;

    /** desiredTurretAngle - measured turret angle, not wrapped. Radians. */
    public final double turretAngleError;

    /**
     * Turret rate that holds the field aim.
     * Field bearing rate minus chassis angular velocity.
     */
    public final double desiredTurretVelocity;

    public final double turretMotorPower;

    /** Chassis yaw rate that walks the turret angle back toward center, rad/s. */
    public final double requestedChassisOmega;

    /** False when no 2π-equivalent aim fits inside the operating window. */
    public final boolean targetReachable;

    /**
     * Latches on past the unwind enter angle and stays on until the command
     * is back inside the exit angle.
     */
    public final boolean unwindActive;

    public TurretCommand(
            double targetBearing,
            double desiredTurretAngle,
            double turretAngleError,
            double desiredTurretVelocity,
            double turretMotorPower,
            double requestedChassisOmega,
            boolean targetReachable,
            boolean unwindActive) {
        this.targetBearing = targetBearing;
        this.desiredTurretAngle = desiredTurretAngle;
        this.turretAngleError = turretAngleError;
        this.desiredTurretVelocity = desiredTurretVelocity;
        this.turretMotorPower = turretMotorPower;
        this.requestedChassisOmega = requestedChassisOmega;
        this.targetReachable = targetReachable;
        this.unwindActive = unwindActive;
    }
}
