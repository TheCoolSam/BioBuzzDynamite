package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Where Pedro AutoTune results go. Every field below is a PLACEHOLDER built from {@link PedroConstants},
 * and none of it is tuned.
 *
 * <p>How to use (full steps in doc/PEDRO_AUTOTUNE_GUIDE.md): run a procedure in AutoTune
 * (http://192.168.43.1:10158), open its Java tab, copy the generated block, and use it to REPLACE the
 * matching field below. The generated blocks are named {@code drivetrainConfig} and
 * {@code localizerConfig} and {@code foresightConfig}. Keep the field NAME shown here (for the localizer,
 * the name that matches your localizer type), so {@link PedroFactory} keeps finding it. Delete this
 * file's placeholder lines for the one you replaced.
 *
 * <p>When all of them are real and the Tests procedure passes, set
 * {@link PedroConstants#HARDWARE_AND_TUNING_CONFIRMED} to true.
 */
public final class PedroTuned {
    private PedroTuned() { }

    // ================================================================ 1. MecanumTuner  ->  drivetrainConfig
    // Replace the whole statement with the generated "drivetrainConfig". Needs the real motor names
    // and the directions the tuner found.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(config -> {
        config.frontLeftName.set(PedroConstants.FRONT_LEFT_MOTOR);
        config.backLeftName.set(PedroConstants.BACK_LEFT_MOTOR);
        config.frontRightName.set(PedroConstants.FRONT_RIGHT_MOTOR);
        config.backRightName.set(PedroConstants.BACK_RIGHT_MOTOR);
        config.frontLeftDirection.set(PedroConstants.FRONT_LEFT_DIRECTION);
        config.backLeftDirection.set(PedroConstants.BACK_LEFT_DIRECTION);
        config.frontRightDirection.set(PedroConstants.FRONT_RIGHT_DIRECTION);
        config.backRightDirection.set(PedroConstants.BACK_RIGHT_DIRECTION);
    });

    // ================================================================ 2. localizer tuner  ->  the ONE you use
    // Use the field that matches PedroConstants.LOCALIZER. Paste the generated "localizerConfig" over it,
    // keeping the field name. The others are ignored.

    /** PinpointTuner output. */
    public static PinpointConfig pinpointConfig = new PinpointConfig(config -> {
        config.name.set(PedroConstants.PINPOINT_NAME);
        // Team believes these are goBILDA swing-arm pods (not yet verified). PinpointTuner asks and overwrites this.
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.xPodOffset.set(PedroConstants.PLACEHOLDER_X_POD_OFFSET_INCHES);
        config.yPodOffset.set(PedroConstants.PLACEHOLDER_Y_POD_OFFSET_INCHES);
        config.offsetUnits.set(DistanceUnit.INCH);
        config.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    /** TwoWheelTuner output. */
    public static TwoWheelConfig twoWheelConfig = new TwoWheelConfig(config -> {
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

    /** ThreeWheelTuner output. */
    public static ThreeWheelConfig threeWheelConfig = new ThreeWheelConfig(config -> {
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

    /** ThreeWheelIMUTuner output. */
    public static ThreeWheelIMUConfig threeWheelImuConfig = new ThreeWheelIMUConfig(config -> {
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

    // ================================================================ 3. ForesightTuner  ->  foresightConfig
    // Placeholder: feedback controllers are zero, so path following will not correct, and the speed/decel
    // stand-ins are 1 in/s. Replace the whole statement with the generated "foresightConfig".
    // Brake matrices are 2x2 because Foresight reads (0,0) and (1,1).
    public static ForesightConfig foresightConfig = new ForesightConfig(values -> {
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
}
