package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Builds one Pedro {@link Follower} for this robot.
 *
 * <p>The OpMode that creates the follower owns its lifecycle. Call
 * {@link Follower#update()} once per loop, then read
 * {@link PedroRobotStateSource}. Nothing else should update the follower
 * or its localizer.
 *
 * <p>Localizer choice is {@link PedroConstants#LOCALIZER}. Drivetrain is a
 * 4-motor mecanum because that is the planned drive. Wheel size, track, and
 * gear ratio are not in this factory: Pedro's mecanum config in 3.0.1 asks
 * for motor names and directions, and the odometry scales live on the
 * localizer. Those values are placeholders in {@link PedroConstants}.
 * Foresight's required fields are set to zeros and 1 in/s stand-ins.
 * That is not a tune.
 */
public final class PedroFactory {

    private PedroFactory() {
    }

    public static Follower create(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("hardwareMap is required");
        }
        return new Follower(localizer(hardwareMap), new Mecanum(hardwareMap, mecanum()), foresight());
    }

    private static Localizer localizer(HardwareMap hardwareMap) {
        switch (PedroConstants.LOCALIZER) {
            case TWO_WHEEL:
                return new TwoWheelLocalizer(hardwareMap, twoWheel());
            case THREE_WHEEL:
                return new ThreeWheelLocalizer(hardwareMap, threeWheel());
            case THREE_WHEEL_IMU:
                return new ThreeWheelIMULocalizer(hardwareMap, threeWheelImu());
            case PINPOINT:
            default:
                return new PinpointLocalizer(hardwareMap, pinpoint());
        }
    }

    private static MecanumConfig mecanum() {
        return new MecanumConfig(config -> {
            config.frontLeftName.set(PedroConstants.FRONT_LEFT_MOTOR);
            config.backLeftName.set(PedroConstants.BACK_LEFT_MOTOR);
            config.frontRightName.set(PedroConstants.FRONT_RIGHT_MOTOR);
            config.backRightName.set(PedroConstants.BACK_RIGHT_MOTOR);
            config.frontLeftDirection.set(PedroConstants.FRONT_LEFT_DIRECTION);
            config.backLeftDirection.set(PedroConstants.BACK_LEFT_DIRECTION);
            config.frontRightDirection.set(PedroConstants.FRONT_RIGHT_DIRECTION);
            config.backRightDirection.set(PedroConstants.BACK_RIGHT_DIRECTION);
        });
    }

    private static PinpointConfig pinpoint() {
        return new PinpointConfig(config -> {
            config.name.set(PedroConstants.PINPOINT_NAME);
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.xPodOffset.set(PedroConstants.PLACEHOLDER_X_POD_OFFSET_INCHES);
            config.yPodOffset.set(PedroConstants.PLACEHOLDER_Y_POD_OFFSET_INCHES);
            config.offsetUnits.set(DistanceUnit.INCH);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
        });
    }

    private static TwoWheelConfig twoWheel() {
        return new TwoWheelConfig(config -> {
            config.xPodName.set(PedroConstants.X_POD_NAME);
            config.yPodName.set(PedroConstants.Y_POD_NAME);
            config.imuName.set(PedroConstants.IMU_NAME);
            config.xPodOffset.set(PedroConstants.PLACEHOLDER_X_POD_OFFSET_INCHES);
            config.yPodOffset.set(PedroConstants.PLACEHOLDER_Y_POD_OFFSET_INCHES);
            config.forwardTicksToInches.set(PedroConstants.PLACEHOLDER_FORWARD_TICKS_TO_INCHES);
            config.strafeTicksToInches.set(PedroConstants.PLACEHOLDER_STRAFE_TICKS_TO_INCHES);
            config.xPodDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.yPodDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.imu.set(new RevHubIMU(PedroConstants.placeholderHubOrientation()));
        });
    }

    private static ThreeWheelConfig threeWheel() {
        return new ThreeWheelConfig(config -> {
            config.leftEncoderName.set(PedroConstants.LEFT_POD_NAME);
            config.rightEncoderName.set(PedroConstants.RIGHT_POD_NAME);
            config.strafeEncoderName.set(PedroConstants.STRAFE_POD_NAME);
            config.leftPodY.set(PedroConstants.PLACEHOLDER_LEFT_POD_Y_INCHES);
            config.rightPodY.set(PedroConstants.PLACEHOLDER_RIGHT_POD_Y_INCHES);
            config.strafePodX.set(PedroConstants.PLACEHOLDER_STRAFE_POD_X_INCHES);
            config.forwardTicksToInches.set(PedroConstants.PLACEHOLDER_FORWARD_TICKS_TO_INCHES);
            config.strafeTicksToInches.set(PedroConstants.PLACEHOLDER_STRAFE_TICKS_TO_INCHES);
            config.turnTicksToRadians.set(PedroConstants.PLACEHOLDER_TURN_TICKS_TO_RADIANS);
            config.leftEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.rightEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.strafeEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
        });
    }

    private static ThreeWheelIMUConfig threeWheelImu() {
        return new ThreeWheelIMUConfig(config -> {
            config.leftEncoderName.set(PedroConstants.LEFT_POD_NAME);
            config.rightEncoderName.set(PedroConstants.RIGHT_POD_NAME);
            config.strafeEncoderName.set(PedroConstants.STRAFE_POD_NAME);
            config.imuName.set(PedroConstants.IMU_NAME);
            config.leftPodY.set(PedroConstants.PLACEHOLDER_LEFT_POD_Y_INCHES);
            config.rightPodY.set(PedroConstants.PLACEHOLDER_RIGHT_POD_Y_INCHES);
            config.strafePodX.set(PedroConstants.PLACEHOLDER_STRAFE_POD_X_INCHES);
            config.forwardTicksToInches.set(PedroConstants.PLACEHOLDER_FORWARD_TICKS_TO_INCHES);
            config.strafeTicksToInches.set(PedroConstants.PLACEHOLDER_STRAFE_TICKS_TO_INCHES);
            config.turnTicksToRadians.set(PedroConstants.PLACEHOLDER_TURN_TICKS_TO_RADIANS);
            config.leftEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.rightEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.strafeEncoderDirection.set(PedroConstants.PLACEHOLDER_ENCODER_DIRECTION);
            config.imu.set(new RevHubIMU(PedroConstants.placeholderHubOrientation()));
        });
    }

    /**
     * Required Foresight fields, filled with stand-ins. Feedback controllers
     * are zero, so path following will not correct until a real tune exists.
     * Brake matrices are 2x2 because {@code getBrakeDisplacement} reads
     * {@code (0,0)} and {@code (1,1)}.
     */
    private static Foresight foresight() {
        ForesightConfig config = new ForesightConfig(values -> {
            values.headingFeedback.set(Controller.proportional(0.0));
            values.forwardTranslational.set(Controller.proportional(0.0));
            values.strafeTranslational.set(Controller.proportional(0.0));
            values.brake.set(Controller.proportional(0.0));
            values.coast.set(Controller.proportional(0.0));
            values.linearBrakeCoefficients.set(Matrix.zero(2));
            values.quadraticBrakeCoefficients.set(Matrix.zero(2));
            values.headingBrakeCoefficients.set(Vector2D.zero());
            values.maxAchievableForwardVelocity.set(PedroConstants.PLACEHOLDER_MAX_FORWARD_IN_PER_SEC);
            values.maxAchievableStrafeVelocity.set(PedroConstants.PLACEHOLDER_MAX_STRAFE_IN_PER_SEC);
            values.naturalForwardDeceleration.set(PedroConstants.PLACEHOLDER_FORWARD_DECEL_IN_PER_SEC2);
            values.naturalStrafeDeceleration.set(PedroConstants.PLACEHOLDER_STRAFE_DECEL_IN_PER_SEC2);
        });
        return new Foresight(config);
    }
}
