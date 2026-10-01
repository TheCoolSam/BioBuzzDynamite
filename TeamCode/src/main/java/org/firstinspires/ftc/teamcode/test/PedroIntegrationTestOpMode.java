package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroFactory;
import org.firstinspires.ftc.teamcode.pedro.PedroManualDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroPoseAdapter;
import org.firstinspires.ftc.teamcode.pedro.PedroRobotStateSource;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateEstimator;

/**
 * Hardware check for the Pedro bridge. Not a driver system and not a tune.
 *
 * <p>One loop updates the follower once, then the estimator. Stick input is
 * only a field-centric request so a sign mistake is visible: stick up is
 * field +Y, stick right is field +X, right stick right is counterclockwise.
 * Releasing the sticks sends zeros. Turret unwind is not connected.
 *
 * <p>If the Control Hub has no matching devices, init does not crash. The
 * no-hardware sign checks live in {@link PedroMappingScenarios}.
 */
@TeleOp(name = "Pedro Integration Test", group = "Test")
public class PedroIntegrationTestOpMode extends OpMode {

    private static final double STICK_DEADBAND = 0.08;

    private final ElapsedTime timer = new ElapsedTime();
    private final RobotStateEstimator estimator = new RobotStateEstimator();

    private Follower follower;
    private PedroRobotStateSource source;
    private boolean ready;
    private String failure = "";

    @Override
    public void init() {
        timer.reset();
        try {
            follower = PedroFactory.create(hardwareMap);
            source = new PedroRobotStateSource(follower);
            ready = true;
            telemetry.addData("Status", "Follower created");
        } catch (RuntimeException e) {
            ready = false;
            failure = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addData("Status", "Hardware not configured");
        }
        telemetry.addData("Localizer placeholder", PedroConstants.LOCALIZER);
        telemetry.addLine("Constants in PedroConstants are not measured.");
    }

    @Override
    public void loop() {
        if (!ready) {
            telemetry.addData("Hardware", "NOT CONFIGURED");
            telemetry.addData("Reason", failure);
            telemetry.addData("Localizer placeholder", PedroConstants.LOCALIZER);
            telemetry.addLine("Fix names and pod geometry in PedroConstants, then run again.");
            telemetry.addLine("Sign checks without hardware: PedroMappingScenarios");
            return;
        }

        double fieldX = deadband(gamepad1.left_stick_x);
        double fieldY = deadband(-gamepad1.left_stick_y);
        double chassisOmega = deadband(gamepad1.right_stick_x);
        boolean commanded = PedroManualDrive.request(follower, fieldX, fieldY, chassisOmega);

        follower.update();
        Pose pedroPose = follower.pose();
        Velocity pedroVelocity = follower.velocity();
        RobotState state = estimator.update(source, timer.seconds());

        publishPedro(pedroPose, pedroVelocity);
        publishState(state);
        telemetry.addData("Drive request", commanded ? "sent" : "refused");
        telemetry.addData("Request X", "%.2f", fieldX);
        telemetry.addData("Request Y", "%.2f", fieldY);
        telemetry.addData("Request omega", "%.2f", chassisOmega);
        telemetry.addLine("Stick up +Y | stick right +X | right stick right CCW");
        telemetry.addLine("Pedro and RobotState columns should match, including sign.");
    }

    private void publishPedro(Pose pose, Velocity velocity) {
        if (pose == null) {
            telemetry.addData("Pedro pose", "null");
        } else {
            telemetry.addData("Pedro X in", "%.2f", pose.x());
            telemetry.addData("Pedro Y in", "%.2f", pose.y());
            telemetry.addData("Pedro heading deg", "%.2f", AngleUtil.toDegrees(pose.heading()));
        }
        if (velocity == null) {
            telemetry.addData("Pedro velocity", "null");
        } else {
            telemetry.addData("Pedro vx", "%.2f", velocity.vx);
            telemetry.addData("Pedro vy", "%.2f", velocity.vy);
            telemetry.addData("Pedro omega", "%.3f", velocity.omega);
        }
        Pose sample = PedroPoseAdapter.toPose(72.0, 24.0, Math.PI / 2.0);
        if (sample != null) {
            telemetry.addData("Adapter sample", "(72, 24, 90 deg) -> (%.1f, %.1f, %.0f deg)",
                    sample.x(), sample.y(), AngleUtil.toDegrees(sample.heading()));
        }
    }

    private void publishState(RobotState state) {
        telemetry.addData("State X in", "%.2f", state.getFieldX());
        telemetry.addData("State Y in", "%.2f", state.getFieldY());
        telemetry.addData("State heading deg", "%.2f", AngleUtil.toDegrees(state.getHeadingRad()));
        telemetry.addData("State vx", "%.2f", state.getFieldVx());
        telemetry.addData("State vy", "%.2f", state.getFieldVy());
        telemetry.addData("State omega", "%.3f", state.getAngularVelocityRadPerSec());
        telemetry.addData("Pose valid", state.isPoseValid() ? "YES" : "NO");
        telemetry.addData("Velocity valid", state.isVelocityValid() ? "YES" : "NO");
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) {
            return 0.0;
        }
        return value;
    }
}
