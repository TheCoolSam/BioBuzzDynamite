package org.firstinspires.ftc.teamcode.turret;

/**
 * One control-cycle snapshot. The controller does not read an OpMode or a gamepad.
 * Fill this from localization, the turret encoder, and the chosen field target.
 *
 * <p>When the robot fields are filled from {@code RobotState}, distances are inches
 * and the target must use inches too. Angles are radians in the
 * {@link org.firstinspires.ftc.teamcode.math.AngleUtil} frame.
 */
public class TurretState {

    public double robotX;
    public double robotY;
    public double robotHeading;

    /** Field-frame robot velocity. Same distance unit as x/y, per second. */
    public double robotVx;
    public double robotVy;

    /** Chassis yaw rate, rad/s. Positive is counterclockwise. */
    public double robotAngularVelocity;
    public double turretAngle;
    public double turretVelocity;
    public double targetX;
    public double targetY;

    /** Seconds since the previous control cycle. Zero or negative is allowed. */
    public double dt;

    /**
     * False when localization has no pose this loop. Defaults true so a state
     * filled by hand still aims. The controller holds position when this is false.
     */
    public boolean poseValid = true;

    /**
     * False when field and yaw rates should not be trusted. Defaults true.
     * Aiming continues, but feedforward is zero.
     */
    public boolean velocityValid = true;
}
