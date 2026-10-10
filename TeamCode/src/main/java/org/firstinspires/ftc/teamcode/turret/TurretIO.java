package org.firstinspires.ftc.teamcode.turret;

/**
 * Hardware seam for the turret motor and absolute encoder.
 * The controller never calls this. RevTurretIO implements the bench hardware path.
 *
 * <p>Angles are chassis-relative radians, counterclockwise positive.
 * Power is positive for counterclockwise motion.
 */
public interface TurretIO {

    double getAngleRad();

    double getVelocityRadPerSec();

    void setMotorPower(double power);

    default void stop() { setMotorPower(0.0); }
}
