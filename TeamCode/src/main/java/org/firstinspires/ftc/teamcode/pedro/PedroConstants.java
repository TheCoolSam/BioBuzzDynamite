package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Every robot-specific Pedro value lives here. None of these are measured.
 *
 * <p>CAD has not fixed wheel size, track, gearing, motor names, motor
 * directions, or odometry pod offsets. The numbers below exist only so the
 * current Pedro config objects can be constructed. They are not tuning.
 * Foresight feedback is left at zero on purpose. Do not copy example
 * coefficients into this file and call the robot tuned.
 */
public final class PedroConstants {

    /**
     * Which localizer {@link PedroFactory} builds. This is not a decision
     * that the hardware has been purchased. Change this one field to move
     * between the localizers Pedro 3.0.1 ships: Pinpoint, two-wheel,
     * three-wheel, or three-wheel plus IMU. Turret, planner, and
     * {@link org.firstinspires.ftc.teamcode.state.RobotState} do not change.
     */
    public static final LocalizerKind LOCALIZER = LocalizerKind.PINPOINT;

    /** Robot Configuration names. Replace when the drivetrain is named. */
    public static final String FRONT_LEFT_MOTOR = "frontLeft";
    public static final String BACK_LEFT_MOTOR = "backLeft";
    public static final String FRONT_RIGHT_MOTOR = "frontRight";
    public static final String BACK_RIGHT_MOTOR = "backRight";

    /**
     * Direction placeholders. FORWARD on all four will not be a working
     * mecanum mix until the motors are wired and checked.
     */
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final String PINPOINT_NAME = "pinpoint";
    public static final String IMU_NAME = "imu";

    /** Inches from the robot center. Zero is "not measured", not a real pod. */
    public static final double PLACEHOLDER_X_POD_OFFSET_INCHES = 0.0;
    public static final double PLACEHOLDER_Y_POD_OFFSET_INCHES = 0.0;
    public static final double PLACEHOLDER_LEFT_POD_Y_INCHES = 0.0;
    public static final double PLACEHOLDER_RIGHT_POD_Y_INCHES = 0.0;
    public static final double PLACEHOLDER_STRAFE_POD_X_INCHES = 0.0;

    public static final String LEFT_POD_NAME = "leftPod";
    public static final String RIGHT_POD_NAME = "rightPod";
    public static final String STRAFE_POD_NAME = "strafePod";
    public static final String X_POD_NAME = "xPod";
    public static final String Y_POD_NAME = "yPod";

    /**
     * Encoder scale placeholders. A real pod is not 1 tick per inch.
     * Replace from the pod datasheet after the track is measured.
     * Signs are +1 until a direction test says otherwise.
     */
    public static final double PLACEHOLDER_FORWARD_TICKS_TO_INCHES = 1.0;
    public static final double PLACEHOLDER_STRAFE_TICKS_TO_INCHES = 1.0;
    public static final double PLACEHOLDER_TURN_TICKS_TO_RADIANS = 1.0;
    public static final double PLACEHOLDER_ENCODER_DIRECTION = 1.0;

    /**
     * Speed and braking limits Pedro requires before a Follower can be built.
     * 1 in/s is not this robot. The pickup planner's test speeds are not
     * these limits either. Replace after a drivetrain exists.
     */
    public static final double PLACEHOLDER_MAX_FORWARD_IN_PER_SEC = 1.0;
    public static final double PLACEHOLDER_MAX_STRAFE_IN_PER_SEC = 1.0;
    public static final double PLACEHOLDER_FORWARD_DECEL_IN_PER_SEC2 = 1.0;
    public static final double PLACEHOLDER_STRAFE_DECEL_IN_PER_SEC2 = 1.0;

    public enum LocalizerKind {
        PINPOINT,
        TWO_WHEEL,
        THREE_WHEEL,
        THREE_WHEEL_IMU
    }

    private PedroConstants() {
    }

    /**
     * Hub logo/USB facing. UP and FORWARD is a stand-in, not a measurement
     * of how this Control Hub is mounted.
     */
    public static RevHubOrientationOnRobot placeholderHubOrientation() {
        return new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}
