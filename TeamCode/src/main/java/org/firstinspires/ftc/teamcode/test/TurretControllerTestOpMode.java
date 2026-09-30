package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.turret.TurretCommand;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.turret.TurretController;
import org.firstinspires.ftc.teamcode.turret.TurretIO;
import org.firstinspires.ftc.teamcode.turret.TurretState;

/**
 * No-hardware turret math check. Nothing in here is a shooter, vision, or drivetrain.
 *
 * <p>The simulated turret is a rate limit, not a motor model:
 * velocity = power * max speed, then integrate, then stop at the physical limits.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: move the simulated robot in the field frame. Stick up is +Y.</li>
 *   <li>Right stick X: rotate the robot. Stick right is positive heading (CCW).</li>
 *   <li>Hold dpad up: spin the robot quickly with the target fixed (disturbance test).</li>
 *   <li>Bumpers: orbit the target around the robot. Right bumper is CCW.</li>
 *   <li>Triggers: slide the target along field X. Right trigger is +X.</li>
 *   <li>Start: toggle applying requested chassis omega to the simulated heading.</li>
 *   <li>A: stationary robot at (0, 0), heading 0, target (1, 0). Expect turret 0.</li>
 *   <li>B: same target, heading +90 deg. Expect turret about -90 deg.</li>
 *   <li>X: turret sitting at +164 deg, target at -164 deg. Must take the long legal path.</li>
 *   <li>Dpad left / right: heading near +179 / -179 deg with the target on +X.</li>
 *   <li>Y: target about +150 deg. Unwind becomes active.</li>
 *   <li>Dpad down: target about +175 deg. That aim is in the rear deadzone.</li>
 * </ul>
 */
@TeleOp(name = "Turret: Tracking Test", group = "Turret")
public class TurretControllerTestOpMode extends OpMode {

    private static final double TRANSLATE_SPEED = 0.8;
    private static final double HEADING_STICK_RATE = Math.toRadians(150.0);
    private static final double SPIN_DEMO_RATE = Math.toRadians(180.0);
    private static final double TARGET_NUDGE_SPEED = 0.6;
    private static final double TARGET_ORBIT_RATE = Math.toRadians(70.0);
    private static final double MAX_SIM_TURRET_SPEED = Math.toRadians(220.0);
    private static final double STICK_DEADBAND = 0.08;
    private static final double MAX_LOOP_DT = 0.05;

    private final ElapsedTime timer = new ElapsedTime();
    private final TurretController controller = new TurretController();
    private final TurretState state = new TurretState();
    private final SimulatedTurretIO turret = new SimulatedTurretIO();

    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private double robotOmega = 0.0;
    private double targetX = 1.0;
    private double targetY = 0.0;

    private boolean applyChassisRequest = false;
    private double lastRequestedChassisOmega = 0.0;
    private String scenario = "A stationary";
    private String scenarioBeforeSpin = "A stationary";

    private boolean aWasDown;
    private boolean bWasDown;
    private boolean xWasDown;
    private boolean yWasDown;
    private boolean dpadLeftWasDown;
    private boolean dpadRightWasDown;
    private boolean dpadDownWasDown;
    private boolean startWasDown;

    @Override
    public void init() {
        applyScenarioA();
        timer.reset();
        telemetry.addData("Status", "No turret hardware required");
    }

    @Override
    public void init_loop() {
        updateFrame();
    }

    @Override
    public void start() {
        timer.reset();
        controller.reset();
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

        boolean preset = readPresetEdges();
        if (!preset) {
            integrateRobot(dt);
            moveTarget(dt);
        }

        state.robotX = robotX;
        state.robotY = robotY;
        state.robotHeading = robotHeading;
        state.robotAngularVelocity = robotOmega;
        state.turretAngle = turret.getAngleRad();
        state.turretVelocity = turret.getVelocityRadPerSec();
        state.targetX = targetX;
        state.targetY = targetY;
        state.dt = dt;

        TurretCommand command = controller.calculate(state);
        lastRequestedChassisOmega = command.requestedChassisOmega;
        turret.setMotorPower(command.turretMotorPower);
        if (!preset) {
            turret.integrate(dt);
        }

        publishTelemetry(command);
    }

    private boolean readPresetEdges() {
        boolean aDown = gamepad1.a;
        boolean bDown = gamepad1.b;
        boolean xDown = gamepad1.x;
        boolean yDown = gamepad1.y;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadDown = gamepad1.dpad_down;
        boolean startDown = gamepad1.start;

        boolean preset = false;
        if (aDown && !aWasDown) {
            applyScenarioA();
            preset = true;
        } else if (bDown && !bWasDown) {
            applyScenarioB();
            preset = true;
        } else if (xDown && !xWasDown) {
            applyWrapScenario();
            preset = true;
        } else if (dpadLeft && !dpadLeftWasDown) {
            applyHeadingWrapScenario(Math.toRadians(179.0));
            preset = true;
        } else if (dpadRight && !dpadRightWasDown) {
            applyHeadingWrapScenario(Math.toRadians(-179.0));
            preset = true;
        } else if (yDown && !yWasDown) {
            applySoftLimitScenario();
            preset = true;
        } else if (dpadDown && !dpadDownWasDown) {
            applyHardLimitScenario();
            preset = true;
        }

        if (startDown && !startWasDown) {
            applyChassisRequest = !applyChassisRequest;
        }

        aWasDown = aDown;
        bWasDown = bDown;
        xWasDown = xDown;
        yWasDown = yDown;
        dpadLeftWasDown = dpadLeft;
        dpadRightWasDown = dpadRight;
        dpadDownWasDown = dpadDown;
        startWasDown = startDown;
        return preset;
    }

    private void integrateRobot(double dt) {
        double stickX = deadband(gamepad1.left_stick_x);
        double stickY = deadband(-gamepad1.left_stick_y);
        robotX += stickX * TRANSLATE_SPEED * dt;
        robotY += stickY * TRANSLATE_SPEED * dt;

        double omega = deadband(gamepad1.right_stick_x) * HEADING_STICK_RATE;
        if (gamepad1.dpad_up) {
            if (!"C spinning".equals(scenario)) {
                scenarioBeforeSpin = scenario;
                scenario = "C spinning";
            }
            omega += SPIN_DEMO_RATE;
        } else if ("C spinning".equals(scenario)) {
            scenario = scenarioBeforeSpin;
        }
        // Previous cycle's request. One loop of delay avoids running the controller twice.
        if (applyChassisRequest) {
            omega += lastRequestedChassisOmega;
        }

        robotHeading = AngleUtil.wrapRadians(robotHeading + omega * dt);
        robotOmega = omega;
    }

    private void moveTarget(double dt) {
        double orbit = 0.0;
        if (gamepad1.right_bumper) {
            orbit += TARGET_ORBIT_RATE;
        }
        if (gamepad1.left_bumper) {
            orbit -= TARGET_ORBIT_RATE;
        }
        if (orbit != 0.0) {
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double radius = Math.hypot(dx, dy);
            if (radius < 0.3) {
                radius = 1.0;
            }
            double bearing = Math.atan2(dy, dx) + (orbit * dt);
            targetX = robotX + (radius * Math.cos(bearing));
            targetY = robotY + (radius * Math.sin(bearing));
        }

        double nudge = gamepad1.right_trigger - gamepad1.left_trigger;
        targetX += nudge * TARGET_NUDGE_SPEED * dt;
    }

    private void applyScenarioA() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
        robotOmega = 0.0;
        targetX = 1.0;
        targetY = 0.0;
        turret.setAngleRad(0.0);
        controller.reset();
        setScenario("A stationary");
    }

    private void applyScenarioB() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = Math.toRadians(90.0);
        robotOmega = 0.0;
        targetX = 1.0;
        targetY = 0.0;
        turret.setAngleRad(0.0);
        controller.reset();
        setScenario("B heading +90");
    }

    private void applyWrapScenario() {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
        robotOmega = 0.0;
        double bearing = Math.toRadians(-164.0);
        targetX = Math.cos(bearing);
        targetY = Math.sin(bearing);
        turret.setAngleRad(Math.toRadians(164.0));
        controller.reset();
        setScenario("D wrap +164 to -164");
    }

    private void applyHeadingWrapScenario(double headingRad) {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = headingRad;
        robotOmega = 0.0;
        targetX = 1.0;
        targetY = 0.0;
        turret.setAngleRad(0.0);
        controller.reset();
        setScenario("D heading " + String.format("%.0f deg", AngleUtil.toDegrees(headingRad)));
    }

    private void applySoftLimitScenario() {
        placeTargetAtBearing(Math.toRadians(150.0));
        turret.setAngleRad(0.0);
        controller.reset();
        setScenario("E soft limit +150");
    }

    private void applyHardLimitScenario() {
        placeTargetAtBearing(Math.toRadians(175.0));
        turret.setAngleRad(Math.toRadians(160.0));
        controller.reset();
        setScenario("F hard limit +175");
    }

    private void setScenario(String name) {
        scenario = name;
        scenarioBeforeSpin = name;
    }

    private void placeTargetAtBearing(double bearingRad) {
        robotX = 0.0;
        robotY = 0.0;
        robotHeading = 0.0;
        robotOmega = 0.0;
        targetX = Math.cos(bearingRad);
        targetY = Math.sin(bearingRad);
    }

    private void publishTelemetry(TurretCommand command) {
        telemetry.addData("Scenario", scenario);
        telemetry.addData("Robot X", "%.3f", robotX);
        telemetry.addData("Robot Y", "%.3f", robotY);
        telemetry.addData("Robot heading deg", "%.2f", AngleUtil.toDegrees(robotHeading));
        telemetry.addData("Robot omega deg/s", "%.1f", AngleUtil.toDegrees(robotOmega));
        telemetry.addData("Target bearing deg", "%.2f", AngleUtil.toDegrees(command.targetBearing));
        telemetry.addData("Desired turret deg", "%.2f", AngleUtil.toDegrees(command.desiredTurretAngle));
        telemetry.addData("Current turret deg", "%.2f", AngleUtil.toDegrees(turret.getAngleRad()));
        telemetry.addData("Turret error deg", "%.2f", AngleUtil.toDegrees(command.turretAngleError));
        telemetry.addData("Requested chassis omega", "%.3f rad/s", command.requestedChassisOmega);
        telemetry.addData("Unwind active", command.unwindActive ? "YES" : "no");
        telemetry.addData("Target reachable", command.targetReachable ? "YES" : "NO");
        telemetry.addData("Turret power", "%.3f", command.turretMotorPower);
        telemetry.addData("Apply chassis omega", applyChassisRequest ? "ON (Start)" : "off (Start)");
        telemetry.addLine("LS move | RS spin | DU hold-spin | LB/RB orbit | LT/RT target X");
        telemetry.addLine("A home | B +90 | X wrap | DL/DR ±179 | Y +150 | DD +175");
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) {
            return 0.0;
        }
        return value;
    }

    /**
     * In-memory turret. Positive power increases the chassis-relative angle.
     * This exists so the geometry can be exercised before a turret is built.
     */
    private static final class SimulatedTurretIO implements TurretIO {
        private double angleRad = 0.0;
        private double velocityRadPerSec = 0.0;
        private double power = 0.0;

        @Override
        public double getAngleRad() {
            return angleRad;
        }

        @Override
        public double getVelocityRadPerSec() {
            return velocityRadPerSec;
        }

        @Override
        public void setMotorPower(double motorPower) {
            power = AngleUtil.clamp(
                    motorPower,
                    TurretConstants.MIN_MOTOR_POWER,
                    TurretConstants.MAX_MOTOR_POWER);
        }

        void setAngleRad(double angle) {
            angleRad = AngleUtil.clamp(
                    angle,
                    TurretConstants.PHYSICAL_MIN_RAD,
                    TurretConstants.PHYSICAL_MAX_RAD);
            velocityRadPerSec = 0.0;
            power = 0.0;
        }

        void integrate(double dt) {
            if (dt <= 0.0) {
                velocityRadPerSec = 0.0;
                return;
            }
            velocityRadPerSec = power * MAX_SIM_TURRET_SPEED;
            double next = angleRad + (velocityRadPerSec * dt);
            if (next >= TurretConstants.PHYSICAL_MAX_RAD) {
                next = TurretConstants.PHYSICAL_MAX_RAD;
                velocityRadPerSec = 0.0;
            } else if (next <= TurretConstants.PHYSICAL_MIN_RAD) {
                next = TurretConstants.PHYSICAL_MIN_RAD;
                velocityRadPerSec = 0.0;
            }
            angleRad = next;
        }
    }
}
