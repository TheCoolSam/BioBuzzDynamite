package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Builds one Pedro {@link Follower} for this robot.
 *
 * <p>The OpMode that creates the follower owns its lifecycle. Call
 * {@link Follower#update()} once per loop, then read
 * {@link PedroRobotStateSource}. Nothing else should update the follower
 * or its localizer.
 *
 * <p>Localizer choice is {@link PedroConstants#LOCALIZER}. Drivetrain is a
 * 4-motor mecanum because that is the planned drive. All configuration
 * objects live in {@link PedroTuned}, which is where Pedro AutoTune results
 * are pasted. Until they are, they are placeholders and not a tune.
 */
public final class PedroFactory {

    private PedroFactory() {
    }

    public static Follower create(HardwareMap hardwareMap) {
        if (!PedroConstants.HARDWARE_AND_TUNING_CONFIRMED) {
            throw new IllegalStateException("Drive/odometry calibration and path tuning required");
        }
        if (hardwareMap == null) {
            throw new IllegalArgumentException("hardwareMap is required");
        }
        return new Follower(localizer(hardwareMap), new Mecanum(hardwareMap, PedroTuned.drivetrainConfig), foresight());
    }

    // ---------------------------------------------------------------- AutoTune access
    // Pedro's AutoTune (see Tuning.java and doc/PEDRO_AUTOTUNE_GUIDE.md) has to build the drivetrain and
    // localizer BEFORE HARDWARE_AND_TUNING_CONFIRMED can honestly be set, so these skip that gate.
    // Nothing else should call them. Competition code must go through create().

    /** AutoTune only. Builds the configured localizer without the confirmation gate. */
    public static Localizer tuningLocalizer(HardwareMap hardwareMap) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap is required");
        return localizer(hardwareMap);
    }

    /** AutoTune only. Builds the mecanum drivetrain without the confirmation gate. */
    public static Drivetrain tuningDrivetrain(HardwareMap hardwareMap) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap is required");
        return new Mecanum(hardwareMap, PedroTuned.drivetrainConfig);
    }

    /** AutoTune only. A Foresight built from whatever {@link PedroTuned#foresightConfig} currently holds. */
    public static Algorithm tuningAlgorithm() {
        return foresight();
    }

    private static Localizer localizer(HardwareMap hardwareMap) {
        switch (PedroConstants.LOCALIZER) {
            case TWO_WHEEL:
                return new TwoWheelLocalizer(hardwareMap, PedroTuned.twoWheelConfig);
            case THREE_WHEEL:
                return new ThreeWheelLocalizer(hardwareMap, PedroTuned.threeWheelConfig);
            case THREE_WHEEL_IMU:
                return new ThreeWheelIMULocalizer(hardwareMap, PedroTuned.threeWheelImuConfig);
            case PINPOINT:
            default:
                return new PinpointLocalizer(hardwareMap, PedroTuned.pinpointConfig);
        }
    }

    private static Foresight foresight() {
        return new Foresight(PedroTuned.foresightConfig);
    }
}
