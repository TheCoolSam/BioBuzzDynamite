package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.robot.FlywheelPair;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.scoring.ShooterCalibrationStorage;

/**
 * Shooter and hood bench tuner.
 *
 * <h3>How the flywheel is tuned (do it in this order)</h3>
 * <ol>
 *   <li><b>OPEN LOOP</b> (starts here, like the OnBot test): Cross/A spins at OPEN_LOOP_POWER.
 *       D-pad up/down changes the power. When the speed stops changing the tuner prints the
 *       measured RPM and a suggested F.</li>
 *   <li>Press <b>Right Stick Click</b> (or tick APPLY_SUGGESTED_PIDF in Dashboard) to load
 *       F = 32767 / measured ticks per second, P = 0.1 F, I = D = 0.</li>
 *   <li>Press <b>Back/Share</b> to switch to VELOCITY mode. Set TARGET_RPM, spin up, then raise
 *       P a little at a time while watching the actualRpm graph and the "last dip" line.</li>
 * </ol>
 *
 * <h3>Dashboard</h3>
 * Every tunable is registered with FTC Dashboard under "Shooter Bench" through an explicit
 * ValueProvider, so the robot knows exactly when a Dashboard edit arrives. The DS screen shows
 * "Dashboard edits received" and the last edit. Gamepad changes are pushed back to the Dashboard
 * panel. In the Dashboard config panel, type the value and press ENTER (or the check mark).
 */
@TeleOp(name = "Shooter & Hood Bench Tuner", group = "Bench")
public class TurretShooterBenchOpMode extends OpMode {

    private static final String DASH_CATEGORY = "Shooter Bench";
    /** Hub velocity controller output saturates at +/-32767, so F ~= 32767 / max ticks per second. */
    private static final double HUB_FULL_SCALE = 32767.0;
    private static final double MAX_RPM_LIMIT = 6500.0;

    private static final double STICK_DEADZONE = 0.15;
    private static final double HOOD_STICK_SPEED = 0.20;
    private static final double COMPRESSION_STICK_SPEED = 0.15;

    private static final double P_STEP = 0.25, F_STEP = 0.25, D_STEP = 0.05, I_STEP = 0.01;
    private static final double RPM_STEP = 50.0, POWER_STEP = 0.05;

    // Axon official PWM frame range (500us - 2500us at 50Hz/20ms frame)
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500, 20000);

    // ---------------------------------------------------------------- tunables (shown in Dashboard)
    private final DashboardTunables dash = new DashboardTunables(DASH_CATEGORY);
    private final DashboardTunables.Num targetRpm = dash.num("TARGET_RPM", 3000.0);
    private final DashboardTunables.Num openLoopPower = dash.num("OPEN_LOOP_POWER", 0.90);
    private final DashboardTunables.Num kP = dash.num("P", 15.0);
    private final DashboardTunables.Num kI = dash.num("I", 15.0);
    private final DashboardTunables.Num kD = dash.num("D", 0.0);
    private final DashboardTunables.Num kF = dash.num("F", 12.5);
    private final DashboardTunables.Num ticksPerRev = dash.num("TICKS_PER_REV", 28.0);
    private final DashboardTunables.Num hoodPosition = dash.num("HOOD_POSITION", 0.35);
    private final DashboardTunables.Num pollenCompression = dash.num("POLLEN_COMPRESSION", 0.30);
    private final DashboardTunables.Num nectarCompression = dash.num("NECTAR_COMPRESSION", 0.50);
    // NECTAR_BALL false = pollen. VELOCITY_MODE false = open loop.
    private final DashboardTunables.Flag nectarBall = dash.flag("NECTAR_BALL", false);
    private final DashboardTunables.Flag velocityMode = dash.flag("VELOCITY_MODE", false);
    private final DashboardTunables.Flag flywheelOn = dash.flag("FLYWHEEL_ON", false);
    // One-shot buttons: tick them in Dashboard; they reset themselves.
    private final DashboardTunables.Flag actApplySuggested = dash.flag("APPLY_SUGGESTED_PIDF", false);
    private final DashboardTunables.Flag actSave = dash.flag("SAVE_CALIBRATION", false);
    private String dashStatus = "NOT CHECKED";

    // ---------------------------------------------------------------- hardware / state
    private MultipleTelemetry multiTelemetry;
    private FlywheelPair flywheel;
    private Servo hood;
    private Servo compression;
    private VoltageSensor battery;

    private boolean servosEngaged = false;
    private String statusMessage = "Ready";

    private final Edge eCross = new Edge(), eSquare = new Edge(), eTriangle = new Edge(),
            eBack = new Edge(), eOptions = new Edge(), eUp = new Edge(), eDown = new Edge(),
            eLeft = new Edge(), eRight = new Edge(), eLb = new Edge(), eRb = new Edge(),
            eRsb = new Edge(), eStop = new Edge();

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    // applied-to-hardware tracking
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;
    private boolean modeApplied = false;
    private boolean appliedVelocityMode = false;
    private double lastHoodCmd = Double.NaN, lastCompCmd = Double.NaN;
    private PIDFCoefficients hubReadback = null;
    private double lastReadbackSec = -10, lastBatterySec = -10, batteryVolts = Double.NaN;

    // measurement
    private double rpmRaw = 0, rpmFiltered = 0, rawTps = 0;
    private double motorAmps = 0, peakAmps = 0, lastCurrentSec = -10;
    private boolean motorOverCurrent = false;
    private double steadyAnchor = Double.NaN, steadySince = 0;
    private boolean steady = false;
    private double steadyRpm = 0, steadyPower = 0, steadyVolts = Double.NaN;
    private double suggestedF = Double.NaN, estMaxRpm = Double.NaN;

    // ball-dip tracker (velocity mode)
    private int dipState = 0;
    private double settledSince = -1, dipStart = 0, dipMin = 0;
    private double lastDipDepthRpm = Double.NaN, lastRecoverySec = Double.NaN;
    private int dipCount = 0;

    private String motorTypeInfo = "unknown";

    private static final class Edge {
        private boolean prev;
        boolean rising(boolean now) {
            boolean r = now && !prev;
            prev = now;
            return r;
        }
    }

    // ================================================================ lifecycle
    @Override
    public void init() {
        FtcDashboard ftcDash = FtcDashboard.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, ftcDash != null ? ftcDash.getTelemetry() : telemetry);
        telemetry.setMsTransmissionInterval(50);

        ShooterCalibrationStorage.Data saved = ShooterCalibrationStorage.load();
        pollenCompression.local(saved.pollenCompression);
        nectarCompression.local(saved.nectarCompression);
        hoodPosition.local(saved.hoodPosition);
        targetRpm.local(saved.targetRpm);
        openLoopPower.local(saved.openLoopPower);
        ticksPerRev.local(saved.ticksPerRev);
        kP.local(saved.p);
        kI.local(saved.i);
        kD.local(saved.d);
        kF.local(saved.f);
        statusMessage = saved.legacyPidfDiscarded
                ? "Old PIDF/RPM numbers from the first bench tuner were discarded (wrong scale)"
                : "Loaded calibration from storage";

        dashStatus = dash.register();

        try {
            flywheel = FlywheelPair.map(hardwareMap, "flywheel", FlywheelPair.SECOND_NAME);
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setPower(0);
            try {
                DcMotorEx m = flywheel.first();
                motorTypeInfo = String.format("%s | %.0f ticks/rev, %.0f rpm, SDK max %.0f ticks/s -> F %.2f",
                        flywheel.status(),
                        m.getMotorType().getTicksPerRev(),
                        m.getMotorType().getMaxRPM(),
                        m.getMotorType().getAchieveableMaxTicksPerSecond(),
                        HUB_FULL_SCALE / m.getMotorType().getAchieveableMaxTicksPerSecond());
            } catch (RuntimeException e) {
                motorTypeInfo = "motor type unreadable";
            }
        } catch (Exception e) {
            statusMessage = "FLYWHEEL NOT FOUND: " + e.getMessage();
        }
        try {
            battery = hardwareMap.voltageSensor.iterator().next();
        } catch (RuntimeException ignored) { }

        hood = initServo("hood");
        compression = initServo("compression");

        clock.reset();
        loopTimer.reset();
        multiTelemetry.addLine("READY. Servos are OFF in INIT; press START to enable.");
        multiTelemetry.addData("Dashboard", dashStatus);
        multiTelemetry.addData("Flywheel motor type", motorTypeInfo);
        multiTelemetry.update();
    }

    @Override
    public void init_loop() {
        multiTelemetry.addData("Dashboard", dashStatus);
        multiTelemetry.addData("Config panel", "Open http://192.168.43.1:8080/dash -> Configuration -> \"%s\"", DASH_CATEGORY);
        multiTelemetry.addData("Flywheel motor type", motorTypeInfo);
        multiTelemetry.addData("Hardware", "flywheel %s | hood %s | compression %s",
                flywheel != null ? "OK" : "MISSING", hood != null ? "OK" : "MISSING",
                compression != null ? "OK" : "MISSING");
        // Prove the Dashboard link BEFORE pressing START: edit any value + Enter, this count must rise.
        multiTelemetry.addData("Dashboard edits received", dash.editSummary());
        multiTelemetry.addData("Status", statusMessage);
        multiTelemetry.update();
        dash.pushIfDirty();
    }

    @Override
    public void start() {
        enableServos();
        servosEngaged = true;
        statusMessage = "Running in " + (velocityMode.v() ? "VELOCITY" : "OPEN LOOP") + " mode";
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.max(0.001, Math.min(0.05, loopTimer.seconds()));
        loopTimer.reset();
        double now = clock.seconds();

        // ---- emergency stop (hold B / Circle). Cuts the flywheel and relaxes servos.
        if (gamepad1.b) {
            if (eStop.rising(true)) {
                flywheelOn.local(false);
                servosEngaged = false;
                disableServos();
                statusMessage = "EMERGENCY STOP (servos relaxed, flywheel off)";
            }
            if (flywheel != null) flywheel.setPower(0);
            measure(now);
            publishTelemetry(now);
            dash.pushIfDirty();
            return;
        }
        eStop.rising(false);

        handleDashboardActions();
        handleGamepad(dt);
        applyServos();
        applyPidfIfChanged();
        applyFlywheel();
        measure(now);
        trackSteadyState(now);
        trackBallDip(now);
        publishTelemetry(now);
        dash.pushIfDirty();
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        disableServos();
        dash.unregister();
    }

    // ================================================================ dashboard actions
    private void handleDashboardActions() {
        if (actApplySuggested.v()) {
            actApplySuggested.local(false);
            applySuggestedPidf();
        }
        if (actSave.v()) {
            actSave.local(false);
            saveCalibration();
        }
        // Dashboard may write nonsense; keep it inside safe ranges.
        targetRpm.local(clamp(targetRpm.v(), 0, MAX_RPM_LIMIT));
        openLoopPower.local(clamp(openLoopPower.v(), 0, 1));
        hoodPosition.local(clamp(hoodPosition.v(), 0, 1));
        pollenCompression.local(clamp(pollenCompression.v(), 0, 1));
        nectarCompression.local(clamp(nectarCompression.v(), 0, 1));
        if (!(ticksPerRev.v() > 0)) ticksPerRev.local(28.0);
        kP.local(Math.max(0, kP.v()));
        kI.local(Math.max(0, kI.v()));
        kD.local(Math.max(0, kD.v()));
        kF.local(Math.max(0, kF.v()));
    }

    // ================================================================ gamepad
    private void handleGamepad(double dt) {
        boolean up = eUp.rising(gamepad1.dpad_up);
        boolean down = eDown.rising(gamepad1.dpad_down);
        boolean left = eLeft.rising(gamepad1.dpad_left);
        boolean right = eRight.rising(gamepad1.dpad_right);

        if (eOptions.rising(gamepad1.options || gamepad1.start)) saveCalibration();

        if (eSquare.rising(gamepad1.square || gamepad1.x)) {
            servosEngaged = !servosEngaged;
            if (servosEngaged) enableServos(); else disableServos();
            statusMessage = servosEngaged ? "Servos engaged" : "Servos relaxed (unpowered)";
        }

        if (eTriangle.rising(gamepad1.triangle || gamepad1.y)) {
            nectarBall.local(!nectarBall.v());
            statusMessage = "Active ball: " + activeBall().name();
        }

        if (eBack.rising(gamepad1.back)) {
            velocityMode.local(!velocityMode.v());
            flywheelOn.local(false);
            statusMessage = "Switched to " + (velocityMode.v() ? "VELOCITY" : "OPEN LOOP") + " mode (flywheel stopped)";
        }

        if (eCross.rising(gamepad1.cross || gamepad1.a)) {
            flywheelOn.local(!flywheelOn.v());
            statusMessage = flywheelOn.v() ? "Flywheel spinning" : "Flywheel stopped";
        }

        if (eRsb.rising(gamepad1.right_stick_button)) applySuggestedPidf();

        boolean holdL2 = gamepad1.left_trigger > 0.5;
        boolean holdR2 = gamepad1.right_trigger > 0.5;
        if (holdL2) {
            // L2 + D-pad: U/D = P, L/R = D
            if (up) kP.local(kP.v() + P_STEP);
            if (down) kP.local(Math.max(0, kP.v() - P_STEP));
            if (right) kD.local(kD.v() + D_STEP);
            if (left) kD.local(Math.max(0, kD.v() - D_STEP));
        } else if (holdR2) {
            // R2 + D-pad: U/D = F, L/R = I
            if (up) kF.local(kF.v() + F_STEP);
            if (down) kF.local(Math.max(0, kF.v() - F_STEP));
            if (right) kI.local(kI.v() + I_STEP);
            if (left) kI.local(Math.max(0, kI.v() - I_STEP));
        } else {
            if (velocityMode.v()) {
                if (up) targetRpm.local(Math.min(MAX_RPM_LIMIT, targetRpm.v() + RPM_STEP));
                if (down) targetRpm.local(Math.max(0, targetRpm.v() - RPM_STEP));
            } else {
                if (up) openLoopPower.local(Math.min(1.0, openLoopPower.v() + POWER_STEP));
                if (down) openLoopPower.local(Math.max(0, openLoopPower.v() - POWER_STEP));
            }
            if (left) adjustCompression(-0.01);
            if (right) adjustCompression(+0.01);
        }

        // Hood: bumpers fine step, right stick continuous (like the OnBot test).
        if (eLb.rising(gamepad1.left_bumper)) hoodPosition.local(clamp(hoodPosition.v() - 0.01, 0, 1));
        if (eRb.rising(gamepad1.right_bumper)) hoodPosition.local(clamp(hoodPosition.v() + 0.01, 0, 1));
        double hoodStick = -gamepad1.right_stick_y;
        if (Math.abs(hoodStick) > STICK_DEADZONE) {
            hoodPosition.local(clamp(hoodPosition.v() + hoodStick * HOOD_STICK_SPEED * dt, 0, 1));
        }
        // Compression: left stick continuous for the active ball.
        double compStick = -gamepad1.left_stick_y;
        if (Math.abs(compStick) > STICK_DEADZONE) {
            adjustCompression(compStick * COMPRESSION_STICK_SPEED * dt);
        }
    }

    // ================================================================ servos
    private Servo initServo(String name) {
        try {
            Servo s = hardwareMap.get(Servo.class, name);
            if (s instanceof PwmControl) {
                ((PwmControl) s).setPwmRange(AXON_PWM_RANGE);
                ((PwmControl) s).setPwmDisable();
            }
            return s;
        } catch (Exception e) {
            statusMessage = name + " servo not found: " + e.getMessage();
            return null;
        }
    }

    private void enableServos() {
        if (hood instanceof PwmControl) ((PwmControl) hood).setPwmEnable();
        if (compression instanceof PwmControl) ((PwmControl) compression).setPwmEnable();
        lastHoodCmd = Double.NaN;
        lastCompCmd = Double.NaN;
        applyServos();
    }

    private void disableServos() {
        if (hood instanceof PwmControl) ((PwmControl) hood).setPwmDisable();
        if (compression instanceof PwmControl) ((PwmControl) compression).setPwmDisable();
        lastHoodCmd = Double.NaN;
        lastCompCmd = Double.NaN;
    }

    private void applyServos() {
        if (!servosEngaged) return;
        if (hood != null && !(Math.abs(hoodPosition.v() - lastHoodCmd) < 1e-4)) {
            hood.setPosition(hoodPosition.v());
            lastHoodCmd = hoodPosition.v();
        }
        double comp = activeCompression();
        if (compression != null && !(Math.abs(comp - lastCompCmd) < 1e-4)) {
            compression.setPosition(comp);
            lastCompCmd = comp;
        }
    }

    private BallType activeBall() { return nectarBall.v() ? BallType.NECTAR : BallType.POLLEN; }

    private double activeCompression() {
        return nectarBall.v() ? nectarCompression.v() : pollenCompression.v();
    }

    private void adjustCompression(double delta) {
        if (nectarBall.v()) nectarCompression.local(clamp(nectarCompression.v() + delta, 0, 1));
        else pollenCompression.local(clamp(pollenCompression.v() + delta, 0, 1));
    }

    // ================================================================ flywheel
    private void applyPidfIfChanged() {
        if (flywheel == null) return;
        double p = kP.v(), i = kI.v(), d = kD.v(), f = kF.v();
        if (p == appliedP && i == appliedI && d == appliedD && f == appliedF) return;
        try {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            appliedP = p; appliedI = i; appliedD = d; appliedF = f;
            lastReadbackSec = -10; // force a fresh readback
        } catch (RuntimeException e) {
            statusMessage = "setPIDFCoefficients failed: " + e.getMessage();
        }
    }

    private void applyFlywheel() {
        if (flywheel == null) return;
        boolean wantVelocity = velocityMode.v();
        if (!modeApplied || wantVelocity != appliedVelocityMode) {
            flywheel.setPower(0);
            flywheel.setMode(wantVelocity ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            appliedVelocityMode = wantVelocity;
            modeApplied = true;
        }
        if (!flywheelOn.v()) {
            flywheel.setPower(0);
        } else if (wantVelocity) {
            flywheel.setVelocity(targetRpm.v() * ticksPerRev.v() / 60.0);
        } else {
            flywheel.setPower(openLoopPower.v());
        }
    }

    private void measure(double now) {
        if (flywheel == null) return;
        rawTps = flywheel.getVelocity();
        rpmRaw = rawTps * 60.0 / ticksPerRev.v();
        rpmFiltered += 0.35 * (rpmRaw - rpmFiltered);
        if (now - lastBatterySec > 0.25 && battery != null) {
            batteryVolts = battery.getVoltage();
            lastBatterySec = now;
        }
        if (now - lastCurrentSec > 0.2) {
            try {
                motorAmps = flywheel.getCurrentAmps();
                motorOverCurrent = flywheel.isOverCurrent();
                peakAmps = Math.max(peakAmps, motorAmps);
            } catch (RuntimeException ignored) { }
            lastCurrentSec = now;
        }
        if (now - lastReadbackSec > 0.5) {
            try { hubReadback = flywheel.first().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER); }
            catch (RuntimeException ignored) { }
            lastReadbackSec = now;
        }
    }

    /** Open loop only: when the speed stops changing, record it and derive F. */
    private void trackSteadyState(double now) {
        boolean running = flywheelOn.v() && !velocityMode.v() && openLoopPower.v() > 0.05 && rpmFiltered > 200;
        if (!running) {
            steadyAnchor = Double.NaN;
            steady = false;
            return;
        }
        if (Double.isNaN(steadyAnchor) || Math.abs(rpmFiltered - steadyAnchor) > 0.015 * steadyAnchor) {
            steadyAnchor = rpmFiltered;
            steadySince = now;
            steady = false;
        } else if (now - steadySince >= 1.5) {
            steady = true;
            steadyRpm = rpmFiltered;
            steadyPower = openLoopPower.v();
            steadyVolts = batteryVolts;
            double tps = steadyRpm * ticksPerRev.v() / 60.0;
            // Normalise to 12 V: the software speed loop scales F by 12 V / battery volts.
            double v = Double.isFinite(steadyVolts) && steadyVolts > 7 ? steadyVolts : 12.0;
            suggestedF = HUB_FULL_SCALE * steadyPower * (v / 12.0) / tps;
            estMaxRpm = steadyRpm / steadyPower;
        }
    }

    /** Velocity mode only: measure how far a ball knocks the speed down and how long recovery takes. */
    private void trackBallDip(double now) {
        double target = targetRpm.v();
        if (!(flywheelOn.v() && velocityMode.v() && target > 200)) {
            dipState = 0;
            settledSince = -1;
            return;
        }
        double err = Math.abs(rpmFiltered - target) / target;
        switch (dipState) {
            case 0:
                if (err < 0.02) {
                    if (settledSince < 0) settledSince = now;
                    if (now - settledSince > 0.5) dipState = 1;
                } else {
                    settledSince = -1;
                }
                break;
            case 1:
                if (rpmFiltered < target * 0.96) {
                    dipState = 2;
                    dipStart = now;
                    dipMin = rpmFiltered;
                }
                break;
            default:
                dipMin = Math.min(dipMin, rpmFiltered);
                if (rpmFiltered >= target * 0.98) {
                    lastDipDepthRpm = target - dipMin;
                    lastRecoverySec = now - dipStart;
                    dipCount++;
                    dipState = 1;
                } else if (now - dipStart > 5.0) {
                    lastDipDepthRpm = target - dipMin;
                    lastRecoverySec = Double.NaN; // never recovered
                    dipCount++;
                    dipState = 0;
                    settledSince = -1;
                }
                break;
        }
    }

    private void applySuggestedPidf() {
        if (!steady || Double.isNaN(suggestedF)) {
            statusMessage = "No steady open-loop reading yet: spin open loop until the speed stops changing";
            return;
        }
        double f = Math.round(suggestedF * 100.0) / 100.0;
        kF.local(f);
        kP.local(Math.round(1.2 * f * 100.0) / 100.0);
        kI.local(Math.round(1.2 * f * 100.0) / 100.0);
        kD.local(0);
        statusMessage = String.format("Applied F=%.2f P=%.2f I=%.2f D=0 from %.0f rpm @ %.0f%% power. Switch to velocity mode (Back).",
                f, kP.v(), kI.v(), steadyRpm, steadyPower * 100);
    }

    private void saveCalibration() {
        ShooterCalibrationStorage.Data data = new ShooterCalibrationStorage.Data();
        data.pollenCompression = pollenCompression.v();
        data.nectarCompression = nectarCompression.v();
        data.hoodPosition = hoodPosition.v();
        data.targetRpm = targetRpm.v();
        data.openLoopPower = openLoopPower.v();
        data.ticksPerRev = ticksPerRev.v();
        data.p = kP.v();
        data.i = kI.v();
        data.d = kD.v();
        data.f = kF.v();
        statusMessage = ShooterCalibrationStorage.save(data)
                ? "CALIBRATION SAVED TO DISK" : "SAVE FAILED (check log)";
    }

    // ================================================================ telemetry
    private void publishTelemetry(double now) {
        double target = targetRpm.v();
        double power = flywheel != null ? flywheel.getPower() : 0;

        // Graph these in Dashboard (Graph tab -> add targetRpm, actualRpm).
        multiTelemetry.addData("targetRpm", flywheelOn.v() && velocityMode.v() ? target : 0.0);
        if (!velocityMode.v()) {
            multiTelemetry.addLine("targetRpm stays 0 in OPEN LOOP: there is no target, the wheel just runs at "
                    + String.format("%.0f%%", openLoopPower.v() * 100) + " power. Press Back/Share for VELOCITY mode.");
        }
        multiTelemetry.addData("actualRpm", Math.round(rpmFiltered));
        multiTelemetry.addData("flywheelPower", String.format("%.3f", power));

        multiTelemetry.addData("MODE", velocityMode.v() ? "VELOCITY (closed loop)" : "OPEN LOOP (like OnBot)");
        multiTelemetry.addData("Flywheel", flywheelOn.v() ? "ON" : "off");
        multiTelemetry.addData("Command", velocityMode.v()
                ? String.format("%.0f rpm", target) : String.format("%.0f%% power", openLoopPower.v() * 100));
        multiTelemetry.addData("Battery V", "%.2f", batteryVolts);
        if (flywheel != null && velocityMode.v()) {
            double out = flywheel.getPower();
            multiTelemetry.addData("Speed loop", "%s, power %.0f%%, I term %+.0f%%%s",
                    FlywheelPair.SOFTWARE_VELOCITY ? "software (battery compensated)" : "hub PIDF",
                    out * 100, flywheel.getIntegralPower() * 100,
                    out > 0.97 ? "  >> PINNED AT FULL POWER: target too high, motor will overheat <<" : "");
        }
        multiTelemetry.addData("Raw rpm (unfiltered)", "%.0f", rpmRaw);
        multiTelemetry.addData("motorAmps", String.format("%.2f", motorAmps));
        multiTelemetry.addData("Motor current (peak this run)", "%.2f A now, %.2f A peak%s", motorAmps, peakAmps,
                motorOverCurrent ? "  >> HUB OVER-CURRENT <<" : "");
        // The hub clamps its velocity output at +/-32767. If F * target already exceeds that, the
        // controller is just pinned at 100% power and the PIDF can no longer help.
        double demandOutput = kF.v() * target * ticksPerRev.v() / 60.0;
        if (velocityMode.v() && flywheelOn.v() && demandOutput > HUB_FULL_SCALE) {
            multiTelemetry.addLine(String.format(
                    ">> SATURATED: F x target asks for %.0f%% power (max 100%%). Lower F, or lower the target. <<",
                    100.0 * demandOutput / HUB_FULL_SCALE));
        }

        multiTelemetry.addLine("--- OPEN LOOP MEASUREMENT ---");
        if (steady) {
            multiTelemetry.addData("STEADY", "%.0f rpm @ %.0f%% power, %.2f V", steadyRpm, steadyPower * 100, steadyVolts);
            multiTelemetry.addData("Suggested F (32767/tps)", "%.2f", suggestedF);
            multiTelemetry.addData("Suggested P and I (1.2 F)", "%.2f", 1.2 * suggestedF);
            multiTelemetry.addData("Rough max rpm @100%", "%.0f", estMaxRpm);
            multiTelemetry.addLine("Right Stick Click = apply these, then Back = velocity mode");
        } else {
            multiTelemetry.addLine("Not steady yet. Open loop, flywheel on, wait for the speed to settle (1.5 s).");
        }

        multiTelemetry.addLine("--- VELOCITY TUNING ---");
        multiTelemetry.addData("PIDF in code", "P %.2f  I %.3f  D %.3f  F %.2f", kP.v(), kI.v(), kD.v(), kF.v());
        if (hubReadback != null) {
            boolean match = Math.abs(hubReadback.p - kP.v()) < 0.01 && Math.abs(hubReadback.i - kI.v()) < 0.01
                    && Math.abs(hubReadback.d - kD.v()) < 0.01 && Math.abs(hubReadback.f - kF.v()) < 0.01;
            multiTelemetry.addData("PIDF on hub", "P %.2f  I %.3f  D %.3f  F %.2f  [%s]",
                    hubReadback.p, hubReadback.i, hubReadback.d, hubReadback.f, match ? "SYNCED" : "MISMATCH");
        }
        multiTelemetry.addData("Error rpm", "%.0f", velocityMode.v() && flywheelOn.v() ? target - rpmFiltered : 0.0);
        multiTelemetry.addData("Last ball dip", Double.isNaN(lastDipDepthRpm) ? "none yet (needs a settled run, then a ball)"
                : String.format("-%.0f rpm, recovered in %s s (%d dips)", lastDipDepthRpm,
                        Double.isNaN(lastRecoverySec) ? "NEVER" : String.format("%.2f", lastRecoverySec), dipCount));
        if (flywheelOn.v() && power > 0.05 && Math.abs(rawTps) < 1) {
            multiTelemetry.addLine(">> WARNING: motor powered but 0 encoder ticks/s. Check the 4-pin encoder cable! <<");
        }
        multiTelemetry.addData("Flywheel motor type", motorTypeInfo);
        multiTelemetry.addData("Encoder ticks/rev used", "%.0f", ticksPerRev.v());

        multiTelemetry.addLine("--- DASHBOARD LINK ---");
        multiTelemetry.addData("Dashboard", dashStatus);
        multiTelemetry.addData("Dashboard edits received", dash.editSummary());

        multiTelemetry.addLine("--- SHOOTER ---");
        multiTelemetry.addData("Active ball", "%s (compression %.3f)", activeBall().name(), activeCompression());
        multiTelemetry.addData("Pollen / Nectar compression", "%.3f / %.3f", pollenCompression.v(), nectarCompression.v());
        multiTelemetry.addData("Hood position", "%.3f", hoodPosition.v());
        multiTelemetry.addData("Servos", servosEngaged ? "ENGAGED" : "RELAXED (unpowered)");
        multiTelemetry.addData("STATUS", statusMessage);

        multiTelemetry.addLine("--- CONTROLS ---");
        multiTelemetry.addLine("A/Cross flywheel | Back mode | D-pad U/D rpm or power | D-pad L/R compression");
        multiTelemetry.addLine("Hold L2 + D-pad: U/D P, L/R D | Hold R2 + D-pad: U/D F, L/R I");
        multiTelemetry.addLine("R-stick hood | L-stick compression | LB/RB hood +/-0.01 | Y ball | X servos");
        multiTelemetry.addLine("R-stick click apply suggested PIDF | Options save | B/Circle E-STOP (hold)");
        multiTelemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
