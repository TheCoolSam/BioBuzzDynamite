package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

/**
 * Pedro Pathing AutoTune procedures. With the app running on the Robot Controller, open
 * <pre>http://192.168.43.1:10158</pre>
 * in a browser on the same Wi-Fi and pick a procedure. Follow doc/PEDRO_AUTOTUNE_GUIDE.md for the order.
 *
 * <p>Each procedure ends by printing Java to paste into {@link PedroConstants} / {@link PedroFactory}.
 * Nothing is applied automatically.
 *
 * <p>Pedro finds {@code @Tuner} methods by scanning, so they must stay public, static, take no
 * arguments and return {@link Procedure}.
 */
public final class Tuning {
    private Tuning() { }

    // ---------------------------------------------------------------- 1. drivetrain
    /** Finds each wheel's direction. Do this first. */
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // ---------------------------------------------------------------- 2. localizer (use the one you have)
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    @Tuner
    public static Procedure threeWheelTuner() {
        return new ThreeWheelTuner();
    }

    @Tuner
    public static Procedure threeWheelImuTuner() {
        return new ThreeWheelIMUTuner();
    }

    @Tuner
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner
    public static Procedure octoQuadTuner() {
        return new OctoQuadTuner();
    }

    // ---------------------------------------------------------------- 3. path following
    /** Measures max speed, braking and gains, then prints a ForesightConfig. Needs steps 1 and 2 pasted in first. */
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(PedroFactory::tuningLocalizer, PedroFactory::tuningDrivetrain);
    }

    // ---------------------------------------------------------------- 4. verify
    /** Odometry, line, curve and hold tests. Needs everything above pasted in. */
    @Tuner
    public static Procedure tests() {
        return new Tests(PedroFactory::tuningDrivetrain, PedroFactory::tuningLocalizer, PedroFactory::tuningAlgorithm);
    }
}
