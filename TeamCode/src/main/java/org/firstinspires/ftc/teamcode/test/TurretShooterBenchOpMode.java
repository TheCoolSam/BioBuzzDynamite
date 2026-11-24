package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.FlywheelPair;
import org.firstinspires.ftc.teamcode.scoring.FlywheelAutoTuner;
import org.firstinspires.ftc.teamcode.scoring.ShooterCalibrationStorage;

/** D-pad jogs the hood and compression. Flywheel and calibration controls live in FTC Dashboard. */
@TeleOp(name = "Shooter & Hood Bench Tuner", group = "Bench")
public class TurretShooterBenchOpMode extends OpMode {

    private static final String DASH_CATEGORY = "Shooter Bench";
    private static final String LOG_TAG = "ShooterBench";
    /** Hub velocity controller output saturates at +/-32767, so F ~= 32767 / max ticks per second. */
    private static final double HUB_FULL_SCALE = 32767.0;

    // Set these to match the robot before running the bench.
    private static final String PRIMARY_FLYWHEEL_NAME = "flywheel";
    private static final String SECOND_FLYWHEEL_NAME = FlywheelPair.SECOND_NAME; // "" for one motor
    private static final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final double MAX_RPM_LIMIT = 6500.0;
    private static final boolean USE_BENCH_SERVOS = true; // false for flywheel-only tuning

    // Rates are normalized servo travel per second.
    private static final double HOOD_JOG_RATE = 0.15;
    private static final double COMPRESSION_JOG_RATE = 0.12;
    private static final double SERVO_COMMAND_RATE = 0.25;

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
    private final DashboardTunables.Flag servosEnabled = dash.flag("SERVOS_ENABLED", true);
    // One-shot buttons: tick them in Dashboard; they reset themselves.
    private final DashboardTunables.Flag actApplySuggested = dash.flag("APPLY_SUGGESTED_PIDF", false);
    private final DashboardTunables.Flag actSave = dash.flag("SAVE_CALIBRATION", false);
    private final DashboardTunables.Flag actAutoTune = dash.flag("AUTO_TUNE_PIDF", false);
    private final DashboardTunables.Flag actStop = dash.flag("STOP_ALL", false);
    private String dashStatus = "NOT CHECKED";

    // ---------------------------------------------------------------- hardware / state
    private Telemetry dashboardTelemetry;
    private FlywheelPair flywheel;
    private Servo hood;
    private Servo compression;
    private VoltageSensor battery;

    private boolean servosEngaged = false;
    private String statusMessage = "Ready";

    // Trial commands stay separate from Dashboard settings and saved calibration.
    private FlywheelAutoTuner autoTune;
    private double tuningTicksPerRev;
    private boolean secondMotorMapped;
    private boolean commandedOn;
    private boolean commandedVelocity;
    private double commandedRpm, commandedPower;

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

    // ================================================================ lifecycle
    @Override
    public void init() {
        FtcDashboard ftcDash = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDash == null ? null : ftcDash.getTelemetry();
        telemetry.setMsTransmissionInterval(100);

        ShooterCalibrationStorage.Data saved = ShooterCalibrationStorage.load();
        pollenCompression.local(saved.pollenCompression);
        nectarCompression.local(saved.nectarCompression);
        hoodPosition.local(saved.hoodPosition);
        savedHood = saved.hoodPosition;
        savedPollen = saved.pollenCompression;
        savedNectar = saved.nectarCompression;
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
            flywheel = FlywheelPair.map(hardwareMap, PRIMARY_FLYWHEEL_NAME, SECOND_FLYWHEEL_NAME);
            secondMotorMapped = flywheel.hasSecond();
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setDirection(FLYWHEEL_DIRECTION);
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

        if (USE_BENCH_SERVOS) {
            hood = initServo("hood");
            compression = initServo("compression");
        }

        clock.reset();
        loopTimer.reset();
        init_loop();
    }

    @Override
    public void init_loop() {
        String hardware = PRIMARY_FLYWHEEL_NAME + (flywheel != null ? " OK" : " MISSING");
        if (USE_BENCH_SERVOS) {
            hardware += String.format(" | hood %s | compression %s",
                    hood != null ? "OK" : "MISSING", compression != null ? "OK" : "MISSING");
        }
        telemetry.addData("Dashboard", dashStatus);
        telemetry.addData("Edits received", dash.editCount());
        telemetry.addData("Hardware", hardware);
        if (USE_BENCH_SERVOS) telemetry.addLine("Servos off in INIT");
        telemetry.update();
        if (dashboardTelemetry != null) {
            dashboardTelemetry.addData("Hardware", hardware);
            dashboardTelemetry.addData("Flywheel motor type", motorTypeInfo);
            dashboardTelemetry.addData("Dashboard edits received", dash.editSummary());
            dashboardTelemetry.addData("Status", statusMessage);
            dashboardTelemetry.update();
        }
        dash.pushIfDirty();
    }

    @Override
    public void start() {
        flywheelOn.local(false);
        actAutoTune.local(false);
        actApplySuggested.local(false);
        actSave.local(false);
        lastHoodCmd = clamp(savedHood, 0, 1);
        lastCompCmd = clamp(savedPollen, 0, 1);
        statusMessage = "Ready";
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.min(0.05, loopTimer.seconds());
        loopTimer.reset();
        double now = clock.seconds();

        if (actStop.v()) {
            actStop.local(false);
            actAutoTune.local(false);
            actApplySuggested.local(false);
            actSave.local(false);
            flywheelOn.local(false);
            servosEnabled.local(false);
            if (tuning()) {
                autoTune.abort("STOP_ALL");
                finishAutoTune();
            } else {
                commandFlywheel(false, false, 0, 0);
            }
            disableServos();
            statusMessage = "Stopped from Dashboard";
        } else {
            clampDashboardSettings();
            if (USE_BENCH_SERVOS) updateServos(dt);
            measure(now);
            if (actAutoTune.v()) {
                actAutoTune.local(false);
                toggleAutoTune(now);
            }
            if (tuning()) {
                flywheelOn.local(false);
                if (actSave.v() || actApplySuggested.v()) {
                    statusMessage = "Wait for auto-tune to finish before applying or saving gains";
                }
                actSave.local(false);
                actApplySuggested.local(false);
                stepAutoTune(now);
            } else {
                if (actApplySuggested.v()) {
                    actApplySuggested.local(false);
                    applySuggestedPidf();
                }
                if (actSave.v()) {
                    actSave.local(false);
                    saveCalibration();
                }
                if (commandedOn && velocityMode.v() != commandedVelocity) flywheelOn.local(false);
                applyManualFlywheel();
            }
            trackSteadyState(now);
            trackBallDip(now);
        }
        publishTelemetry();
        dash.pushIfDirty();
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        disableServos();
        dash.unregister();
    }

    private void clampDashboardSettings() {
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

    private void updateServos(double dt) {
        if (!servosEnabled.v()) {
            if (servosEngaged) disableServos();
            return;
        }
        if (!servosEngaged) enableServos();

        int hoodDirection = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
        if (hoodDirection != 0) {
            hoodPosition.local(clamp(hoodPosition.v() + hoodDirection * HOOD_JOG_RATE * dt, 0, 1));
        }
        int compressionDirection = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
        if (compressionDirection != 0) adjustCompression(compressionDirection * COMPRESSION_JOG_RATE * dt);

        double maxStep = SERVO_COMMAND_RATE * dt;
        lastHoodCmd = moveServo(hood, lastHoodCmd, hoodPosition.v(), maxStep);
        lastCompCmd = moveServo(compression, lastCompCmd, activeCompression(), maxStep);
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

    /** Send a known position before enabling PWM. Servo.getPosition() has no shaft feedback. */
    private void enableServos() {
        if (hood != null) hood.setPosition(lastHoodCmd);
        if (compression != null) compression.setPosition(lastCompCmd);
        if (hood instanceof PwmControl) ((PwmControl) hood).setPwmEnable();
        if (compression instanceof PwmControl) ((PwmControl) compression).setPwmEnable();
        servosEngaged = true;
    }

    private void disableServos() {
        if (hood instanceof PwmControl) ((PwmControl) hood).setPwmDisable();
        if (compression instanceof PwmControl) ((PwmControl) compression).setPwmDisable();
        servosEngaged = false;
    }

    private static double moveServo(Servo servo, double current, double target, double maxStep) {
        if (servo == null || current == target) return current;
        double next = current + clamp(target - current, -maxStep, maxStep);
        servo.setPosition(next);
        return next;
    }

    private String activeBall() { return nectarBall.v() ? "NECTAR" : "POLLEN"; }

    private double activeCompression() {
        return nectarBall.v() ? nectarCompression.v() : pollenCompression.v();
    }

    private void adjustCompression(double delta) {
        if (nectarBall.v()) nectarCompression.local(clamp(nectarCompression.v() + delta, 0, 1));
        else pollenCompression.local(clamp(pollenCompression.v() + delta, 0, 1));
    }

    // ================================================================ flywheel
    private boolean applyPidf(double p, double i, double d, double f) {
        if (p == appliedP && i == appliedI && d == appliedD && f == appliedF) return true;
        try {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
            appliedP = p;
            appliedI = i;
            appliedD = d;
            appliedF = f;
            lastReadbackSec = -10;
            return true;
        } catch (RuntimeException e) {
            statusMessage = "Could not apply PIDF: " + e.getMessage();
            return false;
        }
    }

    private void applyManualFlywheel() {
        boolean useVelocity = velocityMode.v();
        boolean on = flywheelOn.v() && (!useVelocity || targetRpm.v() > 0) && flywheel != null;
        if (on && useVelocity && !applyPidf(kP.v(), kI.v(), kD.v(), kF.v())) {
            flywheelOn.local(false);
            on = false;
        }
        commandFlywheel(on, useVelocity, targetRpm.v(), openLoopPower.v());
    }

    private void commandFlywheel(boolean on, boolean useVelocity, double rpm, double power) {
        commandedOn = on && flywheel != null;
        commandedVelocity = useVelocity;
        commandedRpm = commandedOn && useVelocity ? rpm : 0;
        commandedPower = commandedOn && !useVelocity ? power : 0;
        if (flywheel == null) return;
        if (!modeApplied || useVelocity != appliedVelocityMode) {
            flywheel.setPower(0);
            flywheel.setMode(useVelocity ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            appliedVelocityMode = useVelocity;
            modeApplied = true;
        }
        if (!commandedOn) {
            flywheel.setPower(0);
        } else if (useVelocity) {
            double encoderScale = tuning() ? tuningTicksPerRev : ticksPerRev.v();
            flywheel.setVelocity(rpm * encoderScale / 60.0);
        } else {
            flywheel.setPower(power);
        }
    }

    private void measure(double now) {
        if (flywheel == null) return;
        rawTps = flywheel.getVelocity();
        double encoderScale = tuning() ? tuningTicksPerRev : ticksPerRev.v();
        rpmRaw = rawTps * 60.0 / encoderScale;
        rpmFiltered = Double.isFinite(rpmFiltered) ? rpmFiltered + 0.35 * (rpmRaw - rpmFiltered) : rpmRaw;
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
        if (!FlywheelPair.SOFTWARE_VELOCITY && now - lastReadbackSec > 0.5) {
            try { hubReadback = flywheel.first().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER); }
            catch (RuntimeException ignored) { }
            lastReadbackSec = now;
        }
    }

    /** Open loop only: when the speed stops changing, record it and derive F. */
    private void trackSteadyState(double now) {
        boolean running = !tuning() && commandedOn && !commandedVelocity && commandedPower > 0.05 && rpmFiltered > 200;
        if (!running) {
            steadyAnchor = Double.NaN;
            steady = false;
            return;
        }
        if (commandedPower != steadyPower || Double.isNaN(steadyAnchor)
                || Math.abs(rpmFiltered - steadyAnchor) > 0.015 * steadyAnchor) {
            steadyPower = commandedPower;
            steadyAnchor = rpmFiltered;
            steadySince = now;
            steady = false;
        } else if (now - steadySince >= 1.5) {
            steady = true;
            steadyRpm = rpmFiltered;
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
        double target = commandedRpm;
        if (tuning() || target <= 200) {
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

    // ================================================================ PIDF auto-tune
    private boolean tuning() {
        return autoTune != null && autoTune.running();
    }

    private void toggleAutoTune(double now) {
        if (tuning()) {
            autoTune.abort("cancelled in Dashboard");
            finishAutoTune();
            return;
        }
        if (flywheel == null) {
            statusMessage = "Auto-tune needs the flywheel motor";
            return;
        }
        if (!FlywheelPair.SOFTWARE_VELOCITY) {
            statusMessage = "Auto-tune needs the battery-compensated software velocity loop";
            return;
        }
        if (secondMotorMapped && !flywheel.hasSecond()) {
            statusMessage = flywheel.status();
            return;
        }
        if (commandedOn || flywheelOn.v() || Math.abs(rpmRaw) >= 100) {
            statusMessage = "Turn FLYWHEEL_ON off and let the flywheel stop before auto-tune";
            return;
        }
        try {
            double encoderScale = ticksPerRev.v();
            FlywheelAutoTuner next = new FlywheelAutoTuner(targetRpm.v(), encoderScale, MAX_RPM_LIMIT);
            next.start(now);
            tuningTicksPerRev = encoderScale;
            autoTune = next;
            statusMessage = "Auto-tune running. Keep clear of the flywheel";
        } catch (IllegalArgumentException e) {
            statusMessage = "Auto-tune: " + e.getMessage();
        }
    }

    private void stepAutoTune(double now) {
        if (ticksPerRev.v() != tuningTicksPerRev) {
            autoTune.abort("TICKS_PER_REV changed during the run");
        } else if (motorOverCurrent) {
            autoTune.abort("flywheel over-current");
        } else if (secondMotorMapped && !flywheel.hasSecond()) {
            autoTune.abort(flywheel.status());
        } else if (Math.abs(rpmRaw) > MAX_RPM_LIMIT) {
            autoTune.abort("flywheel passed the RPM limit");
        }
        FlywheelAutoTuner.Output output = autoTune.update(now, rpmFiltered, batteryVolts);
        if (tuning() && output.velocityMode) {
            // Measure each I candidate with a fresh integral instead of inheriting the last trial's correction.
            if (commandedVelocity && output.i != appliedI) flywheel.setPower(0);
            if (!applyPidf(output.p, output.i, 0, output.f)) autoTune.abort(statusMessage);
        }
        if (tuning()) {
            boolean newSweepPower = !output.velocityMode && output.power != commandedPower;
            commandFlywheel(output.on, output.velocityMode, output.targetRpm, output.power);
            if (newSweepPower) {
                RobotLog.ii(LOG_TAG, "Auto-tune commanded %.0f%% power (starting speed %.0f rpm, %.1f V)",
                        output.power * 100, rpmFiltered, batteryVolts);
            }
        } else {
            finishAutoTune();
        }
    }

    private void finishAutoTune() {
        flywheelOn.local(false);
        commandFlywheel(false, false, 0, 0);
        RobotLog.ii(LOG_TAG, "Auto-tune finished: %s", autoTune.message());
        if (autoTune.phase() == FlywheelAutoTuner.Phase.DONE) {
            targetRpm.local(autoTune.targetRpm());
            kP.local(autoTune.resultP());
            kI.local(autoTune.resultI());
            kF.local(autoTune.resultF());
            kD.local(0);
            velocityMode.local(true);
            statusMessage = "Auto-tune complete. Save in Dashboard";
        } else {
            statusMessage = "Auto-tune stopped. See Dashboard";
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
        statusMessage = String.format("Applied F=%.2f P=%.2f I=%.2f D=0 from %.0f rpm @ %.0f%% power. Select VELOCITY_MODE in Dashboard.",
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
        boolean ok = ShooterCalibrationStorage.save(data);
        if (ok) { savedHood = data.hoodPosition; savedPollen = data.pollenCompression; savedNectar = data.nectarCompression; }
        statusMessage = ok ? "Calibration saved to Control Hub" : "SAVE FAILED (check log)";
    }

    /** What is on disk, so the screen can say when hood/compression edits are not saved yet. */
    private double savedHood = Double.NaN, savedPollen = Double.NaN, savedNectar = Double.NaN;

    private boolean servoValuesUnsaved() {
        return Math.abs(hoodPosition.v() - savedHood) > 1e-4 || Math.abs(pollenCompression.v() - savedPollen) > 1e-4
                || Math.abs(nectarCompression.v() - savedNectar) > 1e-4;
    }

    // ================================================================ telemetry
    private void publishTelemetry() {
        if (USE_BENCH_SERVOS) {
            telemetry.addLine("D-pad up/down: hood | left/right: compression");
            telemetry.addData("Hood / compression", "%.3f / %.3f (%s)%s",
                    hoodPosition.v(), activeCompression(), activeBall(), servosEngaged ? "" : " | servos off");
        }
        telemetry.addData("Flywheel", "%s | %.0f rpm",
                tuning() ? "auto-tuning" : commandedOn ? "on" : "off", rpmFiltered);
        telemetry.addData("Status", motorOverCurrent ? "Flywheel over-current" : statusMessage);
        telemetry.addLine("FTC Dashboard > Shooter Bench");
        telemetry.update();

        if (dashboardTelemetry == null) return;
        if (autoTune != null) {
            dashboardTelemetry.addData("Auto-tune", autoTune.message());
            if (autoTune.phase() == FlywheelAutoTuner.Phase.F_SWEEP) {
                dashboardTelemetry.addData("Auto-tune power step elapsed", "%.1f s", autoTune.sweepElapsedSec());
            }
            for (String line : autoTune.pReport()) dashboardTelemetry.addLine(line);
        }
        double power = flywheel != null ? flywheel.getPower() : 0;
        dashboardTelemetry.addData("targetRpm", commandedRpm);
        dashboardTelemetry.addData("actualRpm", rpmFiltered);
        dashboardTelemetry.addData("flywheelPower", power);
        dashboardTelemetry.addData("motorAmps", motorAmps);
        dashboardTelemetry.addData("Flywheel", commandedOn ? (commandedVelocity ? "velocity" : "open loop") : "off");
        dashboardTelemetry.addData("Battery V", "%.2f", batteryVolts);
        dashboardTelemetry.addData("Peak current", "%.2f A%s", peakAmps, motorOverCurrent ? " | HUB OVER-CURRENT" : "");
        dashboardTelemetry.addData("Raw rpm", "%.0f", rpmRaw);
        if (commandedRpm > 0 && flywheel != null) {
            dashboardTelemetry.addData("Speed loop", "%s | I term %+.1f%%%s",
                    FlywheelPair.SOFTWARE_VELOCITY ? "software (battery compensated)" : "hub PIDF",
                    flywheel.getIntegralPower() * 100,
                    power > 0.97 ? " | full power, check RPM headroom" : "");
        }
        if (steady) {
            dashboardTelemetry.addData("STEADY", "%.0f rpm @ %.0f%% power, %.2f V", steadyRpm, steadyPower * 100, steadyVolts);
            dashboardTelemetry.addData("Suggested F", "%.2f", suggestedF);
            dashboardTelemetry.addData("Suggested P and I", "%.2f", 1.2 * suggestedF);
            dashboardTelemetry.addData("Estimated max rpm", "%.0f", estMaxRpm);
        }
        dashboardTelemetry.addData("Configured PIDF", "P %.2f  I %.3f  D %.3f  F %.2f", kP.v(), kI.v(), kD.v(), kF.v());
        if (commandedRpm > 0) {
            dashboardTelemetry.addData("Active PIDF", "P %.2f  I %.3f  D %.3f  F %.2f", appliedP, appliedI, appliedD, appliedF);
            dashboardTelemetry.addData("Error rpm", "%.0f", commandedRpm - rpmFiltered);
        }
        if (hubReadback != null) {
            dashboardTelemetry.addData("PIDF on hub", "P %.2f  I %.3f  D %.3f  F %.2f",
                    hubReadback.p, hubReadback.i, hubReadback.d, hubReadback.f);
        }
        dashboardTelemetry.addData("Last ball dip", Double.isNaN(lastDipDepthRpm) ? "none yet"
                : String.format("-%.0f rpm, recovery %s (%d dips)", lastDipDepthRpm,
                        Double.isNaN(lastRecoverySec) ? "timed out" : String.format("%.2f s", lastRecoverySec), dipCount));
        if (commandedOn && power > 0.05 && Math.abs(rawTps) < 1) {
            dashboardTelemetry.addLine("Motor powered with zero encoder ticks. Check the encoder cable.");
        }
        dashboardTelemetry.addData("Flywheel motor type", motorTypeInfo);
        if (flywheel != null) dashboardTelemetry.addData("Flywheel hardware", flywheel.status());
        dashboardTelemetry.addData("Encoder ticks/rev used", "%.0f", ticksPerRev.v());
        dashboardTelemetry.addData("Dashboard edits received", dash.editSummary());
        if (USE_BENCH_SERVOS) {
            dashboardTelemetry.addData("Active ball", activeBall());
            dashboardTelemetry.addData("Hood command / target", "%.3f / %.3f", lastHoodCmd, hoodPosition.v());
            dashboardTelemetry.addData("Compression command / target", "%.3f / %.3f", lastCompCmd, activeCompression());
            dashboardTelemetry.addData("Servos", servosEngaged ? "enabled" : "off");
            if (servoValuesUnsaved()) dashboardTelemetry.addLine("Use SAVE_CALIBRATION to keep hood and compression changes.");
        }
        dashboardTelemetry.addData("Status", statusMessage);
        dashboardTelemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
