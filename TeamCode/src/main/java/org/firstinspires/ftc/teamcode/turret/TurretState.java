package org.firstinspires.ftc.teamcode.turret;

/**
 * One control-cycle snapshot. The controller does not read an OpMode or a gamepad.
 * Fill this from localization, the turret encoder, and the chosen field target.
 *
 * <p>Distances are in whatever field unit localization uses, as long as robot and
 * target positions share that unit. Angles are radians in the
 * {@link org.firstinspires.ftc.teamcode.math.AngleUtil} frame.
 */
public class TurretState {

    public double robotX;
    public double robotY;
    public double robotHeading;
    public double robotAngularVelocity;
    public double turretAngle;
    public double turretVelocity;
    public double targetX;
    public double targetY;

    /** Seconds since the previous control cycle. Zero or negative is allowed. */
    public double dt;
}
