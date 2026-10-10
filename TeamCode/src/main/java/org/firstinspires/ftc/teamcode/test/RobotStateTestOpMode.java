package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.state.RobotStateSource;
import org.firstinspires.ftc.teamcode.turret.TurretCommand;
import org.firstinspires.ftc.teamcode.turret.TurretController;
import org.firstinspires.ftc.teamcode.turret.TurretState;
import org.firstinspires.ftc.teamcode.turret.TurretStateAdapter;

/**
 * No-hardware check of the robot snapshot. Nothing here drives a robot.
 *
 * <p>The simulated source integrates velocity into pose. The estimator only
 * copies that motion and differences velocity to get acceleration.
 * A fixed target at (72 in, 0) is handed to the existing turret controller
 * through {@link TurretStateAdapter}.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: field velocity. Stick up is +Y, inches per second.</li>
 *   <li>Right stick X: angular velocity. Stick right is counterclockwise.</li>
 *   <li>A: constant +24 in/s along +X. Acceleration should settle at 0.</li>
 *   <li>B: constant +1 rad/s rotation. Angular acceleration should settle at 0.</li>
 *   <li>X: both of those at once.</li>
 *   <li>Y: step +X speed up by 24 in/s. One loop shows a large Ax, then it returns to 0.</li>
 *   <li>Dpad left: heading +179 deg at +1 rad/s, so heading passes through ±180.</li>
 *   <li>Dpad down: toggle a pose-invalid reading.</li>
 *   <li>Dpad up: one loop of non-finite velocity.</li>
 *   <li>Left bumper: reuse the previous timestamp so dt is not positive.</li>
 *   <li>Start: reset the simulated robot and the estimator.</li>
 * </ul>
 */
@TeleOp(name = "Robot State Test", group = "Test")
public class RobotStateTestOpMode extends OpMode {

    private static final double STICK_SPEED_IN_PER_SEC = 48.0;
    private static final double STICK_OMEGA = Math.toRadians(120.0);
    private static final double TRANSLATE_IN_PER_SEC = 24.0;
    private static final double ROTATE_RAD_PER_SEC = 1.0;
    private static final double TARGET_X_INCHES = 72.0;
    private static final double MAX_LOOP_DT = 0.05;
    private static final double STICK_DEADBAND = 0.08;

    private final ElapsedTime timer = new ElapsedTime();
    private final RobotStateEstimator estimator = new RobotStateEstimator();
    private final SimulatedSource source = new SimulatedSource();
    private final TurretController turretController = new TurretController();

    private double timestampSec = 0.0;
    private String scenario = "manual";
    private boolean poseForcedInvalid = false;

    private boolean aWasDown;
    private boolean bWasDown;
    private boolean xWasDown;
    private boolean yWasDown;
    private boolean dpadLeftWasDown;
    private boolean dpadDownWasDown;
    private boolean dpadUpWasDown;
    private boolean startWasDown;

    @Override
    public void init() {
        timer.reset();
        telemetry.addData("Status", "No hardware required");
    }

    @Override
    public void init_loop() {
        updateFrame();
    }

    @Override
    public void start() {
        timer.reset();
        estimator.reset();
        turretController.reset();
    }

    @Override
    public void loop() {
        updateFrame();
    }

    private void updateFrame() {
        double dt = timer.seconds();
        timer.reset();
        if (dt < 0.0 || dt > MAX_LOOP_DT) {
            dt = 0.02;
        }

        readButtons();
        applyManualMotion();

        source.poseValid = !poseForcedInvalid;
        boolean holdTimestamp = gamepad1.left_bumper;
        if (!holdTimestamp) {
            timestampSec += dt;
            source.integrate(dt);
        }

        source.acquiredSec=timestampSec;
        RobotState robot = estimator.update(source, timestampSec);
        TurretState turretState = TurretStateAdapter.toTurretState(
                robot,
                0.0,
                0.0,
                TARGET_X_INCHES,
                0.0,
                holdTimestamp ? 0.0 : dt);
        TurretCommand turretCommand = turretController.calculate(turretState);

        publish(robot, turretCommand, holdTimestamp);
    }

    private void readButtons() {
        if (pressed(gamepad1.a, aWasDown)) {
            source.vx = TRANSLATE_IN_PER_SEC;
            source.vy = 0.0;
            source.omega = 0.0;
            estimator.reset();
            scenario = "constant translation";
        }
        aWasDown = gamepad1.a;

        if (pressed(gamepad1.b, bWasDown)) {
            source.vx = 0.0;
            source.vy = 0.0;
            source.omega = ROTATE_RAD_PER_SEC;
            estimator.reset();
            scenario = "constant rotation";
        }
        bWasDown = gamepad1.b;

        if (pressed(gamepad1.x, xWasDown)) {
            source.vx = TRANSLATE_IN_PER_SEC;
            source.vy = 0.0;
            source.omega = ROTATE_RAD_PER_SEC;
            estimator.reset();
            scenario = "translation + rotation";
        }
        xWasDown = gamepad1.x;

        if (pressed(gamepad1.y, yWasDown)) {
            source.vx += TRANSLATE_IN_PER_SEC;
            scenario = "velocity step";
        }
        yWasDown = gamepad1.y;

        if (pressed(gamepad1.dpad_left, dpadLeftWasDown)) {
            source.heading = Math.toRadians(179.0);
            source.omega = ROTATE_RAD_PER_SEC;
            source.vx = 0.0;
            source.vy = 0.0;
            estimator.reset();
            scenario = "heading wrap";
        }
        dpadLeftWasDown = gamepad1.dpad_left;

        if (pressed(gamepad1.dpad_down, dpadDownWasDown)) {
            poseForcedInvalid = !poseForcedInvalid;
        }
        dpadDownWasDown = gamepad1.dpad_down;

        if (pressed(gamepad1.dpad_up, dpadUpWasDown)) {
            source.reportNonFiniteVx = true;
            scenario = "non-finite velocity";
        }
        dpadUpWasDown = gamepad1.dpad_up;

        if (pressed(gamepad1.start, startWasDown)) {
            source.reset();
            estimator.reset();
            turretController.reset();
            timestampSec = 0.0;
            poseForcedInvalid = false;
            scenario = "manual";
        }
        startWasDown = gamepad1.start;
    }

    private void applyManualMotion() {
        if (!"manual".equals(scenario)) {
            return;
        }
        double stickX = deadband(gamepad1.left_stick_x);
        double stickY = deadband(-gamepad1.left_stick_y);
        source.vx = stickX * STICK_SPEED_IN_PER_SEC;
        source.vy = stickY * STICK_SPEED_IN_PER_SEC;
        source.omega = deadband(gamepad1.right_stick_x) * STICK_OMEGA;
    }

    private void publish(RobotState robot, TurretCommand turretCommand, boolean holdTimestamp) {
        telemetry.addData("Scenario", scenario);
        telemetry.addData("Timestamp s", "%.3f", robot.getTimestampSec());
        telemetry.addData("X in", "%.2f", robot.getFieldX());
        telemetry.addData("Y in", "%.2f", robot.getFieldY());
        telemetry.addData("Heading deg", "%.2f", AngleUtil.toDegrees(robot.getHeadingRad()));
        telemetry.addData("Vx in/s", "%.2f", robot.getFieldVx());
        telemetry.addData("Vy in/s", "%.2f", robot.getFieldVy());
        telemetry.addData("Omega rad/s", "%.3f", robot.getAngularVelocityRadPerSec());
        telemetry.addData("Ax in/s^2", "%.1f", robot.getFieldAx());
        telemetry.addData("Ay in/s^2", "%.1f", robot.getFieldAy());
        telemetry.addData("Alpha rad/s^2", "%.2f", robot.getAngularAccelerationRadPerSec2());
        telemetry.addData("Pose valid", robot.isPoseValid() ? "YES" : "NO");
        telemetry.addData("Velocity valid", robot.isVelocityValid() ? "YES" : "NO");
        telemetry.addData("Hold timestamp", holdTimestamp ? "YES" : "no");
        telemetry.addData("Turret desired deg", "%.2f", AngleUtil.toDegrees(turretCommand.desiredTurretAngle));
        telemetry.addData("Turret tracking", turretCommand.tracking ? "YES" : "NO");
        telemetry.addLine("LS velocity | RS spin | A translate | B rotate | X both");
        telemetry.addLine("Y step vx | DL wrap | DD pose invalid | DU NaN vel | LB bad dt | Start reset");
    }

    private static boolean pressed(boolean down, boolean wasDown) {
        return down && !wasDown;
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) {
            return 0.0;
        }
        return value;
    }

    /**
     * In-memory localization. The estimator does not integrate this;
     * the OpMode does, the same way a real pose source would.
     */
    private static final class SimulatedSource implements RobotStateSource {
        private double acquiredSec;
        @Override public double getAcquisitionTimestampSec() {return acquiredSec;}
        @Override public boolean isDeviceHealthy() {return true;}
        private double x = 0.0;
        private double y = 0.0;
        private double heading = 0.0;
        private double vx = 0.0;
        private double vy = 0.0;
        private double omega = 0.0;
        private boolean poseValid = true;
        private boolean reportNonFiniteVx = false;

        void reset() {
            x = 0.0;
            y = 0.0;
            heading = 0.0;
            vx = 0.0;
            vy = 0.0;
            omega = 0.0;
            poseValid = true;
        }

        void integrate(double dt) {
            if (dt <= 0.0 || !poseValid) {
                return;
            }
            x += vx * dt;
            y += vy * dt;
            heading = AngleUtil.wrapRadians(heading + (omega * dt));
        }

        @Override
        public double getX() {
            return x;
        }

        @Override
        public double getY() {
            return y;
        }

        @Override
        public double getHeadingRad() {
            return heading;
        }

        @Override
        public double getFieldVx() {
            if (reportNonFiniteVx) {
                reportNonFiniteVx = false;
                return Double.NaN;
            }
            return vx;
        }

        @Override
        public double getFieldVy() {
            return vy;
        }

        @Override
        public double getAngularVelocityRadPerSec() {
            return omega;
        }

        @Override
        public boolean isPoseValid() {
            return poseValid;
        }

        @Override
        public boolean isVelocityValid() {
            return true;
        }
    }
}
