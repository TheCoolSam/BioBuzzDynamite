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

    /** Placeholder physical stop, about -165 degrees. */
    public static final double PHYSICAL_MIN_RAD = Math.toRadians(-165.0);

    /** Placeholder physical stop, about +165 degrees. Final safety clamp only. */
    public static final double PHYSICAL_MAX_RAD = Math.toRadians(165.0);

    /**
     * Lowest turret angle the controller will intentionally command.
     * Inside the physical stop by about 5 degrees. Placeholder, about -160 degrees.
     */
    public static final double OPERATING_MIN_RAD = Math.toRadians(-160.0);

    /**
     * Highest turret angle the controller will intentionally command.
     * Inside the physical stop by about 5 degrees. Placeholder, about +160 degrees.
     */
    public static final double OPERATING_MAX_RAD = Math.toRadians(160.0);

    /**
     * Unwind latches on once the commanded turret angle passes this.
     * About 140 degrees. Once latched, recovery continues inside the exit angle.
     */
    public static final double UNWIND_ENTER_RAD = Math.toRadians(140.0);

    /**
     * Unwind stays latched until target geometry and measured turret are inside this.
     * About 130 degrees. Stops the request from flickering around the enter angle.
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
