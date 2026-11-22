package org.firstinspires.ftc.teamcode.turret;

/**
 * Placeholder turret limits and untuned gains. Stored values are radians.
 * The degree numbers below are the CAD-facing placeholders and are converted
 * once, here. Do not copy degree literals into the controller.
 *
 * <p>Angles use the frame documented on
 * {@link org.firstinspires.ftc.teamcode.math.AngleUtil}.
 *
 * <p>kP is only large enough for the no-hardware test OpMode to track.
 * kD and kV stay at 0 until the real motor, gear ratio, and encoder exist.
 * Replace every value in this class after CAD. Do not treat them as match gains.
 */
public final class TurretConstants {

    // Yaw limits. CAD sweep of Ragnar Chassis Rev E v283 (Nov 19 2026), turret 0 = CAD pose
    // (pointing forward), positive = CCW seen from above. After the flower pivot cross-tube was
    // split into two stubs, nothing on the chassis blocks the turret anywhere in 360 degrees with
    // the flower arms stowed or tilted outward (top of the arms away from the robot, any angle).
    // Closest passes, stowed: Limelight mount bolt heads ~1 mm from the right flower arm near
    // -5 degrees and from the left arm near -110 degrees.
    // Arms tilted back over the robot (top toward the turret) DO hit the turret: never do that.
    // So these limits are set by the turret wiring, not by collisions. They are only right
    // once TurretHardwareConfig.ENCODER_ZERO_OFFSET_RAD is calibrated so 0 = straight forward.

    /** Final safety clamp: no power further than this (wiring limit). */
    public static final double PHYSICAL_MIN_RAD = Math.toRadians(-165.0);

    /** Final safety clamp: no power further than this (wiring limit). */
    public static final double PHYSICAL_MAX_RAD = Math.toRadians(165.0);

    /** Lowest turret angle the controller will intentionally command (CW). */
    public static final double OPERATING_MIN_RAD = Math.toRadians(-160.0);

    /** Highest turret angle the controller will intentionally command (CCW). */
    public static final double OPERATING_MAX_RAD = Math.toRadians(160.0);

    /**
     * Unwind (turn the chassis instead) latches once the commanded turret angle passes this on
     * either side. Must be below the smaller operating limit.
     */
    public static final double UNWIND_ENTER_RAD = Math.toRadians(140.0);

    /**
     * Unwind stays latched until target geometry and measured turret are inside this.
     * Stops the request from flickering around the enter angle.
     */
    public static final double UNWIND_EXIT_RAD = Math.toRadians(130.0);

    /**
     * Chassis radians per second requested per radian of turret angle past
     * the interior recovery target below {@link #UNWIND_EXIT_RAD}. Placeholder.
     */
    public static final double UNWIND_KP = 3.0;

    /** Largest chassis yaw rate this controller will request, in rad/s. Placeholder. */
    public static final double MAX_REQUESTED_CHASSIS_OMEGA = 2.0;

    /** Limits increases in requested yaw; rad/s per second. Placeholder, untuned. */
    public static final double UNWIND_OMEGA_RAMP_RAD_PER_SEC2 = 4.0;

    /** Position gain, power per radian of turret error. Placeholder, not tuned. */
    public static final double KP = 2.0;

    /** Derivative gain, power per (rad/s) of turret error rate. Leave at 0 until tuned. */
    public static final double KD = 0.0;

    /**
     * Feedforward gain, power per (rad/s) of desired turret velocity.
     * The feedforward path is wired. Leave the gain at 0 until the turret is characterized.
     */
    public static final double KV = 0.0;

    public static final double MIN_MOTOR_POWER = -1.0;
    public static final double MAX_MOTOR_POWER = 1.0;

    private TurretConstants() {
    }
}
