package org.firstinspires.ftc.teamcode.turret;

/**
 * Hardware seam for the turret motor and absolute encoder.
 * The controller never calls this. A future RevTurretIO can implement it
 * once the motor, gear ratio, and encoder mount are known.
 *
 * <p>Angles are chassis-relative radians, counterclockwise positive.
 * Power is positive for counterclockwise motion.
 */
public interface TurretIO {

    double getAngleRad();

    double getVelocityRadPerSec();

    void setMotorPower(double power);
}
