package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.FlywheelReadyDetector;
import org.firstinspires.ftc.teamcode.auto.HeadingHold;
import org.firstinspires.ftc.teamcode.auto.SimpleAutoPlan;
import org.firstinspires.ftc.teamcode.auto.SimpleAutoRunner;
import org.firstinspires.ftc.teamcode.auto.SimpleAutoStorage;
import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.match.MatchSetup;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.robot.BasicDrive;
import org.firstinspires.ftc.teamcode.robot.FlywheelPair;
import org.firstinspires.ftc.teamcode.scoring.ShooterCalibrationStorage;
import org.firstinspires.ftc.teamcode.test.DashboardTunables;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Simple Auto: an autonomous that needs only what is physically on the robot. No Pedro, no turret
 * gate, no sensors, no odometry. Drive legs are timed from speeds you measure.
 *
 * <p>Routes (pick with D-pad up/down in INIT, or ROUTE in the Dashboard):
 * 0 leave the zone, 1 shoot the preload then park, 2 preload + pickup + second shot + park.
 * X = blue, B = red in INIT. Blue mirrors every strafe.
 *
 * <p>Everything tunable is in the Dashboard panel "Simple Auto". Press OPTIONS to save the auto-specific
 * values to the Control Hub (they survive INIT, STOP and power cycles). Shooter values come from the
 * bench tuner's saved file, so tune the shooter on the bench first.
 *
 * <p>Safety: B stops everything and latches. The auto ends all outputs before 30 s. If time is short
 * it abandons shooting and parks.
 *
 * <p>NOTHING here has been run on the real robot. Verify wheel directions, then mechanism directions,
 * with the robot on blocks, before the first floor run (see doc/SIMPLE_AUTO_GUIDE.md).
 */
@Autonomous(name = "BIOBUZZ Simple Auto", group = "Competition", preselectTeleOp = "BIOBUZZ TeleOp")
public final class BioBuzzSimpleAuto extends OpMode {
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500, 20000);
    private static final String[] ROUTE_NAMES = {"0 LEAVE ONLY", "1 PRELOAD SHOT + PARK", "2 PRELOAD + PICKUP + SECOND SHOT + PARK"};

    // ---------------------------------------------------------------- tunables
    private final DashboardTunables dash = new DashboardTunables("Simple Auto");
    private final Map<String, DashboardTunables.Num> persistNums = new LinkedHashMap<String, DashboardTunables.Num>();
    private final Map<String, DashboardTunables.Flag> persistFlags = new LinkedHashMap<String, DashboardTunables.Flag>();

    private DashboardTunables.Num num(String name, double def, boolean persist) {
        DashboardTunables.Num n = dash.num(name, def);
        if (persist) persistNums.put(name, n);
        return n;
    }

    private DashboardTunables.Flag flag(String name, boolean def, boolean persist) {
        DashboardTunables.Flag f = dash.flag(name, def);
        if (persist) persistFlags.put(name, f);
        return f;
    }

    // Route
    private final DashboardTunables.Num route = num("ROUTE", 1, true);
    private final DashboardTunables.Flag blue = flag("BLUE_ALLIANCE", false, true);
    /**
     * Pit only, robot on blocks: lets the INIT wheel test spin the drive motors. Never saved, always
     * starts OFF, so a stray LB/RB/RT press while lining up at a match cannot move the robot in INIT.
     */
    private final DashboardTunables.Flag pitWheelTest = flag("PIT_WHEEL_TEST", false, false);
    private final DashboardTunables.Num startDelay = num("START_DELAY_SEC", 0.0, true);
    private final DashboardTunables.Num shots = num("SHOTS", 3, true);
    // Drive
    private final DashboardTunables.Num drivePower = num("DRIVE_POWER", 0.40, true);
    private final DashboardTunables.Num ipsForward = num("IN_PER_SEC_FORWARD", 30.0, true);
    private final DashboardTunables.Num ipsStrafe = num("IN_PER_SEC_STRAFE", 24.0, true);
    private final DashboardTunables.Num slew = num("SLEW_PER_SEC", 3.0, true);
    private final DashboardTunables.Num shotMoveFwd = num("SHOT_MOVE_FORWARD_IN", 0.0, true);
    private final DashboardTunables.Num shotMoveStrafe = num("SHOT_MOVE_STRAFE_IN", 0.0, true);
    private final DashboardTunables.Num pickupFwd = num("PICKUP_FORWARD_IN", 24.0, true);
    private final DashboardTunables.Num pickupStrafe = num("PICKUP_STRAFE_IN", 0.0, true);
    private final DashboardTunables.Num pickupSettle = num("PICKUP_SETTLE_SEC", 0.3, true);
    private final DashboardTunables.Num parkFwd = num("PARK_FORWARD_IN", 30.0, true);
    private final DashboardTunables.Num parkStrafe = num("PARK_STRAFE_IN", 0.0, true);
    private final DashboardTunables.Flag revFL = flag("REVERSE_FRONT_LEFT", true, true);
    private final DashboardTunables.Flag revBL = flag("REVERSE_BACK_LEFT", true, true);
    private final DashboardTunables.Flag revFR = flag("REVERSE_FRONT_RIGHT", false, true);
    private final DashboardTunables.Flag revBR = flag("REVERSE_BACK_RIGHT", false, true);
    // Heading hold: keeps the robot pointed straight while a leg drives. OFF until you verify the IMU (guide step 2).
    private final DashboardTunables.Flag headingHold = flag("HEADING_HOLD", false, true);
    private final DashboardTunables.Num headingKp = num("HEADING_KP", 0.02, true);
    private final DashboardTunables.Num headingMax = num("HEADING_MAX", 0.20, true);
    private final DashboardTunables.Num headingFaultDeg = num("HEADING_FAULT_DEG", 20.0, true);
    private final DashboardTunables.Flag headingReverse = flag("REVERSE_HEADING", false, true);
    // Control Hub mounting. Each is 0 UP, 1 DOWN, 2 FORWARD, 3 BACKWARD, 4 LEFT, 5 RIGHT.
    private final DashboardTunables.Num imuLogo = num("IMU_LOGO_FACING", 0, true);
    private final DashboardTunables.Num imuUsb = num("IMU_USB_FACING", 2, true);
    // Speed calibration (after a run: type what you measured, then tick APPLY, then press OPTIONS)
    private final DashboardTunables.Num measuredFwd = num("MEASURED_FORWARD_IN", 0.0, false);
    private final DashboardTunables.Num measuredStrafe = num("MEASURED_STRAFE_IN", 0.0, false);
    private final DashboardTunables.Flag applyMeasured = flag("APPLY_MEASURED_SPEEDS", false, false);
    // Shooter (defaults come from the bench tuner's file; not saved here)
    private final DashboardTunables.Flag openLoop = flag("OPEN_LOOP", true, true);
    private final DashboardTunables.Flag useEncoder = flag("USE_ENCODER_FOR_READY", true, true);
    private final DashboardTunables.Num shotPower = num("SHOT_POWER", 0.30, false);
    private final DashboardTunables.Num shotRpm = num("SHOT_RPM", 3000, false);
    private final DashboardTunables.Num kP = num("P", 15.0, false);
    private final DashboardTunables.Num kI = num("I", 15.0, false);
    private final DashboardTunables.Num kD = num("D", 0.0, false);
    private final DashboardTunables.Num kF = num("F", 12.5, false);
    private final DashboardTunables.Num ticksPerRev = num("TICKS_PER_REV", 28.0, false);
    private final DashboardTunables.Num hoodPos = num("HOOD", 0.35, false);
    private final DashboardTunables.Num compressionPos = num("COMPRESSION", 0.30, false);
    // Shot timing
    private final DashboardTunables.Num minSpinup = num("MIN_SPINUP_SEC", 1.0, true);
    private final DashboardTunables.Num timedSpinup = num("TIMED_SPINUP_SEC", 3.0, true);
    private final DashboardTunables.Num spinupMax = num("SPINUP_MAX_SEC", 4.0, true);
    private final DashboardTunables.Num feedPulse = num("FEED_PULSE_SEC", 0.35, true);
    private final DashboardTunables.Num minRecover = num("MIN_RECOVER_SEC", 0.15, true);
    private final DashboardTunables.Num recoverMax = num("RECOVER_MAX_SEC", 1.5, true);
    // Mechanisms
    private final DashboardTunables.Num feederPower = num("FEEDER_POWER", 1.0, true);
    private final DashboardTunables.Num transferPower = num("TRANSFER_POWER", 1.0, true);
    private final DashboardTunables.Num intakePower = num("INTAKE_POWER", 1.0, true);
    private final DashboardTunables.Flag revFeeder = flag("REVERSE_FEEDER", false, true);
    private final DashboardTunables.Flag revTransfer = flag("REVERSE_TRANSFER", false, true);
    private final DashboardTunables.Flag revIntake = flag("REVERSE_INTAKE", false, true);
    // Deadlines
    private final DashboardTunables.Num parkBy = num("PARK_BY_SEC", 25.5, true);
    private final DashboardTunables.Num noNewShotAfter = num("NO_NEW_SHOT_AFTER_SEC", 24.0, true);

    // ---------------------------------------------------------------- hardware
    private final DcMotor[] drive = new DcMotor[4]; // FL, BL, FR, BR
    private final boolean[] appliedReverse = new boolean[4];
    private final boolean[] appliedReverseKnown = new boolean[4];
    private FlywheelPair flywheel;
    private DcMotor intake, transfer;
    /** Final feeder: goBILDA dual-mode servo in continuous mode. */
    private CRServo feeder;
    private Servo hood, compression;
    private VoltageSensor battery;
    private IMU imu;
    private int imuLogoApplied = -1, imuUsbApplied = -1;
    private String imuStatus = "not checked";
    private HeadingHold hold;
    private double lastHeadingDeg = Double.NaN;
    private double lastHoldYaw;
    private final List<String> hardwareProblems = new ArrayList<String>();

    // ---------------------------------------------------------------- run state
    private MultipleTelemetry multi;
    private String dashStatus = "";
    private String storageStatus = "";
    private String saveStatus = "";
    private final ElapsedTime loopTimer = new ElapsedTime();
    private SimpleAutoRunner runner;
    private FlywheelReadyDetector detector;
    private List<SimpleAutoPlan.Step> runPlan;
    private SimpleAutoRunner.Params runParams;
    private String configError = null;
    private boolean estop;
    private boolean finished;
    private double lastRpm;
    private boolean lastFeed;
    private final List<String> shotLog = new ArrayList<String>();
    private SimpleAutoRunner.Command lastCommand = new SimpleAutoRunner.Command();
    private boolean flywheelWasOn;
    private boolean servosEnabled;
    private double lastPidfSig = Double.NaN;
    private DcMotor.RunMode flywheelMode;
    private double lastFlywheelSetpoint = Double.NaN;
    private boolean prevUp, prevDown, prevOptions;
    private double lastDt = 0.02;
    private double finishedAt = Double.NaN;
    /** B selects RED in INIT; if it is still held at START it must be released before it can e-stop. */
    private boolean bMustRelease;

    // ================================================================ lifecycle
    @Override
    public void init() {
        multi = new MultipleTelemetry(telemetry, FtcDashboard.getInstance() != null ? FtcDashboard.getInstance().getTelemetry() : telemetry);

        // Shooter defaults come from the bench tuner's file so bench tuning carries over.
        ShooterCalibrationStorage.Data shooter = ShooterCalibrationStorage.load();
        shotPower.local(shooter.openLoopPower);
        shotRpm.local(shooter.targetRpm);
        kP.local(shooter.p);
        kI.local(shooter.i);
        kD.local(shooter.d);
        kF.local(shooter.f);
        ticksPerRev.local(shooter.ticksPerRev);
        hoodPos.local(shooter.hoodPosition);
        compressionPos.local(shooter.pollenCompression);

        loadSaved();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        String[] names = {PedroConstants.FRONT_LEFT_MOTOR, PedroConstants.BACK_LEFT_MOTOR,
                PedroConstants.FRONT_RIGHT_MOTOR, PedroConstants.BACK_RIGHT_MOTOR};
        for (int i = 0; i < 4; i++) {
            drive[i] = motor(names[i]);
            if (drive[i] != null) {
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive[i].setPower(0);
            }
        }
        applyDriveDirections();

        try {
            flywheel = FlywheelPair.map(hardwareMap, "flywheel", FlywheelPair.SECOND_NAME);
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setPower(0);
            flywheelMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            if (!flywheel.hasSecond()) hardwareProblems.add("flywheel: " + flywheel.status());
        } catch (Exception e) {
            hardwareProblems.add("flywheel not found");
        }
        intake = motor("intake");
        transfer = motor("transfer");
        feeder = crServo("feeder");
        hood = servo("hood");
        compression = servo("compression");
        try { battery = hardwareMap.voltageSensor.iterator().next(); } catch (RuntimeException ignored) { }
        try { imu = hardwareMap.get(IMU.class, "imu"); } catch (Exception e) { imu = null; imuStatus = "no device named \"imu\""; }
        applyImuOrientation();

        dashStatus = dash.register();
        loopTimer.reset();
        multi.addLine("BIOBUZZ Simple Auto ready. Pick alliance (X blue / B red) and route (D-pad up/down).");
        multi.update();
    }

    @Override
    public void init_loop() {
        // Alliance and route from the gamepad. Held LB is the wheel test, so the buttons do not double up.
        boolean lb = gamepad1.left_bumper && pitWheelTest.v();
        if (!lb) {
            if (gamepad1.x) blue.local(true);
            if (gamepad1.b) blue.local(false);
        }
        // Carry the pick into TeleOp (same buttons there: X blue, B red).
        MatchSetup.setAlliance(blue.v() ? AllianceColor.BLUE : AllianceColor.RED);
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        if (up && !prevUp) route.local(Math.min(2, Math.round(route.v()) + 1));
        if (down && !prevDown) route.local(Math.max(0, Math.round(route.v()) - 1));
        prevUp = up;
        prevDown = down;
        handleSave();

        applyDriveDirections();
        applyImuOrientation();
        wheelTest();

        SimpleAutoPlan.Config cfg = planConfig();
        SimpleAutoRunner.Params params = runnerParams();
        List<String> problems = SimpleAutoPlan.validate(cfg);

        multi.addLine("=== INIT: choose, check, then press START ===");
        multi.addData("Alliance (X blue / B red)", blue.v() ? "BLUE" : "RED");
        multi.addData("Route (D-pad up/down)", routeName((int) Math.round(route.v())));
        multi.addData("Start delay / shots", "%.1f s / %d", startDelay.v(), (int) Math.round(shots.v()));
        if (!problems.isEmpty()) {
            for (String p : problems) multi.addData("PROBLEM", p);
            multi.addLine("Fix the problem(s) above in the Dashboard. START will not move the robot until the plan is valid.");
        } else {
            List<SimpleAutoPlan.Step> steps = SimpleAutoPlan.build(cfg);
            for (SimpleAutoPlan.Step s : steps) multi.addData(stepIcon(s), describe(s));
            double best = SimpleAutoPlan.bestCaseSeconds(steps, params);
            double worst = SimpleAutoPlan.worstCaseSeconds(steps, params);
            multi.addData("Plan time best / worst", "%.1f s / %.1f s (limit 30)", best, worst);
            if (worst > params.parkByElapsedSec) {
                multi.addData("WARNING", "worst case passes the park-by time (%.1f s). If shots time out, the second shot or pickup may be dropped.", params.parkByElapsedSec);
            }
        }
        for (String h : hardwareProblems) multi.addData("HARDWARE", h);
        multi.addData("Flywheel mode", openLoop.v()
                ? String.format(Locale.US, "OPEN LOOP power %.2f (ready: %s)", shotPower.v(), useEncoder.v() ? "encoder plateau" : "fixed timer")
                : String.format(Locale.US, "VELOCITY %.0f rpm, PIDF %.2f/%.2f/%.2f/%.2f", shotRpm.v(), kP.v(), kI.v(), kD.v(), kF.v()));
        multi.addData("Hood / compression", "%.2f / %.2f (from bench file)", hoodPos.v(), compressionPos.v());
        multi.addData("Heading hold", headingHold.v() ? "ON" : "off (verify the IMU, then tick HEADING_HOLD)");
        multi.addData("IMU", imuStatus + (imu != null && Double.isFinite(readHeading()) ? String.format(Locale.US, " | yaw %.1f deg (turn the robot counterclockwise by hand: it must INCREASE)", lastHeadingDeg) : ""));
        multi.addLine(pitWheelTest.v()
                ? ">> PIT WHEEL TEST ON (robot on blocks!): hold LB + X=FL Y=FR A=BL B=BR, or RB=all forward, RT=strafe right. Turn PIT_WHEEL_TEST off before a match."
                : "Nothing moves in INIT. (Wheel test: Dashboard PIT_WHEEL_TEST, pit only.)");
        multi.addLine("OPTIONS saves auto tuning to the Control Hub.");
        multi.addData("Saved values", storageStatus.isEmpty() ? "none loaded (code defaults)" : storageStatus);
        if (!saveStatus.isEmpty()) multi.addData("Last save", saveStatus);
        multi.addData("Dashboard", dashStatus + " | " + dash.editSummary());
        dash.pushIfDirty();
        multi.update();
    }

    @Override
    public void start() {
        bMustRelease = gamepad1.b;
        stopAll();
        SimpleAutoPlan.Config cfg = planConfig();
        runParams = runnerParams();
        List<String> problems = SimpleAutoPlan.validate(cfg);
        if (!problems.isEmpty()) {
            configError = problems.get(0);
            return;
        }
        try {
            runPlan = SimpleAutoPlan.build(cfg);
            runner = new SimpleAutoRunner(runPlan, runParams);
        } catch (RuntimeException e) {
            configError = e.getMessage();
            return;
        }
        detector = new FlywheelReadyDetector(detectorConfig());
        hold = null;
        if (headingHold.v()) {
            if (imu == null || imuStatus.startsWith("ERROR") || imuStatus.startsWith("no device")) {
                imuStatus = "HEADING HOLD REQUESTED BUT IMU UNUSABLE: driving without it";
            } else {
                try {
                    HeadingHold.Config hc = new HeadingHold.Config();
                    hc.kP = headingKp.v();
                    hc.maxCorrection = headingMax.v();
                    hc.faultDeg = headingFaultDeg.v();
                    hc.reversed = headingReverse.v();
                    hold = new HeadingHold(hc);
                    imu.resetYaw();
                    hold.setTarget(0);
                } catch (RuntimeException e) {
                    hold = null;
                    imuStatus = "HEADING HOLD DISABLED: " + e.getMessage();
                }
            }
        }
        enableServos();
        applyServoPositions();
        applyDriveDirections();
        applyMechanismDirections();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.max(0.0, Math.min(0.1, loopTimer.seconds()));
        loopTimer.reset();
        lastDt = dt;
        handleSave();

        if (!gamepad1.b) bMustRelease = false;
        if (gamepad1.b && !bMustRelease) estop = true;
        if (configError != null || runner == null || estop || finished) {
            stopAll();
            if (!estop && finished) handleCalibration();
            showRunTelemetry();
            return;
        }

        double rpm = readRpm();
        detector.update(dt, rpm, lastCommand.flywheelOn);
        SimpleAutoRunner.Command cmd = runner.update(dt, detector.ready());
        lastCommand = cmd;

        if (cmd.done) {
            finished = true;
            finishedAt = runner.elapsed();
            stopAll();
            showRunTelemetry();
            return;
        }

        applyServoPositions();
        lastHoldYaw = 0;
        if (hold != null && (cmd.x != 0 || cmd.y != 0)) {
            lastHoldYaw = hold.correction(readHeading());
        }
        applyDrive(cmd, lastHoldYaw);
        applyFlywheel(cmd.flywheelOn);
        applyMechanisms(cmd);

        if (cmd.feed && !lastFeed) {
            shotLog.add(String.format(Locale.US, "#%d t=%.1fs %.0f rpm", shotLog.size() + 1, runner.elapsed(), rpm));
        }
        lastFeed = cmd.feed;
        showRunTelemetry();
    }

    @Override
    public void stop() {
        stopAll();
        dash.unregister();
    }

    // ================================================================ configuration
    private SimpleAutoPlan.Config planConfig() {
        SimpleAutoPlan.Config c = new SimpleAutoPlan.Config();
        c.route = (int) Math.round(route.v());
        c.blue = blue.v();
        c.startDelaySec = startDelay.v();
        c.shots = (int) Math.round(shots.v());
        c.inPerSecForward = ipsForward.v();
        c.inPerSecStrafe = ipsStrafe.v();
        c.shotMoveForwardIn = shotMoveFwd.v();
        c.shotMoveStrafeIn = shotMoveStrafe.v();
        c.pickupForwardIn = pickupFwd.v();
        c.pickupStrafeIn = pickupStrafe.v();
        c.pickupSettleSec = pickupSettle.v();
        c.parkForwardIn = parkFwd.v();
        c.parkStrafeIn = parkStrafe.v();
        return c;
    }

    private SimpleAutoRunner.Params runnerParams() {
        SimpleAutoRunner.Params p = new SimpleAutoRunner.Params();
        p.slewPerSec = slew.v() > 0 ? slew.v() : p.slewPerSec;
        p.spinupMaxSec = spinupMax.v();
        p.feedPulseSec = feedPulse.v();
        p.minRecoverSec = minRecover.v();
        p.recoverMaxSec = recoverMax.v();
        p.parkByElapsedSec = parkBy.v();
        p.noNewShotAfterSec = noNewShotAfter.v();
        return p;
    }

    private FlywheelReadyDetector.Config detectorConfig() {
        FlywheelReadyDetector.Config d = new FlywheelReadyDetector.Config();
        if (!openLoop.v()) {
            d.mode = FlywheelReadyDetector.Mode.VELOCITY;
            d.targetRpm = shotRpm.v();
        } else {
            d.mode = useEncoder.v() ? FlywheelReadyDetector.Mode.OPEN_LOOP : FlywheelReadyDetector.Mode.TIMED;
        }
        d.minOnSec = minSpinup.v();
        d.timedSpinupSec = timedSpinup.v();
        return d;
    }

    // ================================================================ saving
    private void loadSaved() {
        Map<String, String> saved = SimpleAutoStorage.load();
        int used = 0;
        for (Map.Entry<String, DashboardTunables.Num> e : persistNums.entrySet()) {
            String v = saved.get(e.getKey());
            if (v == null) continue;
            try {
                double d = Double.parseDouble(v);
                if (!Double.isNaN(d) && !Double.isInfinite(d)) { e.getValue().local(d); used++; }
            } catch (NumberFormatException ignored) { }
        }
        for (Map.Entry<String, DashboardTunables.Flag> e : persistFlags.entrySet()) {
            String v = saved.get(e.getKey());
            if (v == null) continue;
            e.getValue().local(Boolean.parseBoolean(v));
            used++;
        }
        storageStatus = saved.isEmpty() ? "" : "loaded " + used + " values from " + SimpleAutoStorage.FILE_NAME;
    }

    private void handleSave() {
        boolean options = gamepad1.options;
        if (options && !prevOptions) {
            Map<String, String> out = new LinkedHashMap<String, String>();
            for (Map.Entry<String, DashboardTunables.Num> e : persistNums.entrySet()) out.put(e.getKey(), Double.toString(e.getValue().v()));
            for (Map.Entry<String, DashboardTunables.Flag> e : persistFlags.entrySet()) out.put(e.getKey(), Boolean.toString(e.getValue().v()));
            saveStatus = SimpleAutoStorage.save(out) ? "SAVED " + out.size() + " values" : "SAVE FAILED (see logcat)";
        }
        prevOptions = options;
    }

    /**
     * After a run that only drove straight, the measured distance gives the real speed:
     * speed = distance / full-power seconds driven. Calibrate with route 0.
     */
    private void handleCalibration() {
        if (!applyMeasured.v()) return;
        applyMeasured.local(false);
        double fwdSec = runner.forwardEquivalentSeconds();
        double strSec = runner.strafeEquivalentSeconds();
        if (measuredFwd.v() > 0 && fwdSec > 0.2) ipsForward.local(round1(measuredFwd.v() / fwdSec));
        if (measuredStrafe.v() > 0 && strSec > 0.2) ipsStrafe.local(round1(measuredStrafe.v() / strSec));
        saveStatus = "Applied measured speeds. Press OPTIONS to save them.";
    }

    private static double round1(double v) { return Math.round(v * 10.0) / 10.0; }

    // ================================================================ hardware helpers
    private DcMotor motor(String name) {
        try {
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            m.setPower(0);
            return m;
        } catch (Exception e) {
            hardwareProblems.add("motor \"" + name + "\" not found");
            return null;
        }
    }

    private CRServo crServo(String name) {
        try {
            CRServo s = hardwareMap.get(CRServo.class, name);
            s.setPower(0);
            return s;
        } catch (Exception e) {
            hardwareProblems.add("continuous servo \"" + name + "\" not found (configure it as a Continuous Rotation Servo)");
            return null;
        }
    }

    private Servo servo(String name) {
        try {
            Servo s = hardwareMap.get(Servo.class, name);
            if (s instanceof PwmControl) {
                ((PwmControl) s).setPwmRange(AXON_PWM_RANGE);
                ((PwmControl) s).setPwmDisable();
            }
            return s;
        } catch (Exception e) {
            hardwareProblems.add("servo \"" + name + "\" not found");
            return null;
        }
    }

    private void enableServos() {
        if (hood instanceof PwmControl) ((PwmControl) hood).setPwmEnable();
        if (compression instanceof PwmControl) ((PwmControl) compression).setPwmEnable();
        servosEnabled = true;
    }

    private void applyServoPositions() {
        if (!servosEnabled) return;
        if (hood != null) hood.setPosition(clamp01(hoodPos.v()));
        if (compression != null) compression.setPosition(clamp01(compressionPos.v()));
    }

    private void applyDriveDirections() {
        boolean[] want = {revFL.v(), revBL.v(), revFR.v(), revBR.v()};
        for (int i = 0; i < 4; i++) {
            if (drive[i] == null) continue;
            if (!appliedReverseKnown[i] || appliedReverse[i] != want[i]) {
                drive[i].setDirection(want[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                appliedReverse[i] = want[i];
                appliedReverseKnown[i] = true;
            }
        }
    }

    private void applyMechanismDirections() {
        if (intake != null) intake.setDirection(revIntake.v() ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (transfer != null) transfer.setDirection(revTransfer.v() ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (feeder != null) feeder.setDirection(revFeeder.v() ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /** Pit-only: spin wheels forward one at a time so wrong directions are obvious before a floor run. */
    private void wheelTest() {
        double[] p = new double[4];
        final double power = 0.30;
        if (!pitWheelTest.v()) {
            // Match safety: nothing moves in INIT unless the pit-only switch is on in the Dashboard.
        } else if (gamepad1.left_bumper) {
            if (gamepad1.x) p[0] = power;
            if (gamepad1.y) p[2] = power;
            if (gamepad1.a) p[1] = power;
            if (gamepad1.b) p[3] = power;
        } else if (gamepad1.right_bumper) {
            double[] m = BasicDrive.powers(0, 1, 0, power);
            if (m != null) p = m;
        } else if (gamepad1.right_trigger > 0.5) {
            double[] m = BasicDrive.powers(1, 0, 0, power);
            if (m != null) p = m;
        }
        for (int i = 0; i < 4; i++) if (drive[i] != null) drive[i].setPower(p[i]);
    }

    /** (Re)initialises the IMU when the mounting numbers change. Only in INIT; never while driving. */
    private void applyImuOrientation() {
        if (imu == null) return;
        int logo = (int) Math.round(imuLogo.v());
        int usb = (int) Math.round(imuUsb.v());
        if (logo == imuLogoApplied && usb == imuUsbApplied) return;
        imuLogoApplied = logo;
        imuUsbApplied = usb;
        try {
            RevHubOrientationOnRobot.LogoFacingDirection[] logos = {
                    RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT};
            RevHubOrientationOnRobot.UsbFacingDirection[] usbs = {
                    RevHubOrientationOnRobot.UsbFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT};
            if (logo < 0 || logo > 5 || usb < 0 || usb > 5) throw new IllegalArgumentException("IMU facing values must be 0 to 5");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logos[logo], usbs[usb])));
            imuStatus = "ok (logo " + logo + ", usb " + usb + ")";
        } catch (RuntimeException e) {
            imuStatus = "ERROR: " + e.getMessage() + " (logo and USB must face different, perpendicular directions)";
        }
    }

    private double readHeading() {
        if (imu == null) return Double.NaN;
        try {
            lastHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } catch (RuntimeException e) {
            lastHeadingDeg = Double.NaN;
        }
        return lastHeadingDeg;
    }

    private double readRpm() {
        if (flywheel == null) return 0;
        try {
            double tpr = ticksPerRev.v() > 0 ? ticksPerRev.v() : 28.0;
            lastRpm = flywheel.getVelocity() * 60.0 / tpr;
        } catch (RuntimeException e) {
            lastRpm = 0;
        }
        return lastRpm;
    }

    private void applyDrive(SimpleAutoRunner.Command cmd, double holdYaw) {
        double cap = Math.max(0.0, Math.min(1.0, drivePower.v()));
        double yaw = Math.max(-1.0, Math.min(1.0, cmd.yaw + holdYaw));
        double[] p = cap > 0 ? BasicDrive.powers(cmd.x, cmd.y, yaw, cap) : null;
        if (p == null) { stopDrive(); return; }
        for (int i = 0; i < 4; i++) if (drive[i] != null) drive[i].setPower(p[i]);
    }

    private void applyFlywheel(boolean on) {
        if (flywheel == null) return;
        if (!on) {
            if (flywheelWasOn) setFlywheelOff();
            flywheelWasOn = false;
            return;
        }
        flywheelWasOn = true;
        if (openLoop.v()) {
            if (flywheelMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheelMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                lastFlywheelSetpoint = Double.NaN;
            }
            double power = clamp01(shotPower.v());
            if (power != lastFlywheelSetpoint) { flywheel.setPower(power); lastFlywheelSetpoint = power; }
        } else {
            if (flywheelMode != DcMotor.RunMode.RUN_USING_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheelMode = DcMotor.RunMode.RUN_USING_ENCODER;
                lastFlywheelSetpoint = Double.NaN;
                lastPidfSig = Double.NaN;
            }
            double sig = kP.v() * 1e6 + kI.v() * 1e4 + kD.v() * 1e2 + kF.v();
            if (sig != lastPidfSig) {
                try {
                    flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP.v(), kI.v(), kD.v(), kF.v()));
                } catch (RuntimeException ignored) { }
                lastPidfSig = sig;
            }
            double tps = shotRpm.v() * (ticksPerRev.v() > 0 ? ticksPerRev.v() : 28.0) / 60.0;
            // Every loop: FlywheelPair runs the battery-compensated speed loop one step per call.
            flywheel.setVelocity(tps);
            lastFlywheelSetpoint = tps;
        }
    }

    private void setFlywheelOff() {
        if (flywheel == null) return;
        try {
            if (flywheelMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheelMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            }
            flywheel.setPower(0);
        } catch (RuntimeException ignored) { }
        lastFlywheelSetpoint = Double.NaN;
    }

    private void applyMechanisms(SimpleAutoRunner.Command cmd) {
        if (intake != null) intake.setPower(cmd.intake ? clamp(intakePower.v()) : 0);
        double feed = cmd.feed ? 1.0 : 0.0;
        if (transfer != null) transfer.setPower(feed * clamp(transferPower.v()));
        if (feeder != null) feeder.setPower(feed * clamp(feederPower.v()));
    }

    private void stopDrive() {
        for (DcMotor m : drive) if (m != null) { try { m.setPower(0); } catch (RuntimeException ignored) { } }
    }

    private void stopAll() {
        stopDrive();
        setFlywheelOff();
        flywheelWasOn = false;
        for (DcMotor m : new DcMotor[]{intake, transfer}) {
            if (m != null) { try { m.setPower(0); } catch (RuntimeException ignored) { } }
        }
        if (feeder != null) { try { feeder.setPower(0); } catch (RuntimeException ignored) { } }
    }

    private static double clamp01(double v) { return Double.isNaN(v) ? 0 : Math.max(0, Math.min(1, v)); }
    private static double clamp(double v) { return Double.isNaN(v) ? 0 : Math.max(-1, Math.min(1, v)); }

    // ================================================================ telemetry
    private void showRunTelemetry() {
        multi.addLine("=== BIOBUZZ SIMPLE AUTO ===");
        if (configError != null) {
            multi.addData("NOT RUNNING", configError);
            multi.addData("Dashboard", dashStatus + " | " + dash.editSummary());
            dash.pushIfDirty();
            multi.update();
            return;
        }
        if (estop) multi.addData("STATE", "E-STOPPED (B). Everything is off.");
        else if (finished) multi.addData("STATE", "FINISHED at " + String.format(Locale.US, "%.1f s", finishedAt));
        else multi.addData("STATE", "RUNNING");
        if (runner != null) {
            multi.addData("Time", "%.1f / 30 s", runner.elapsed());
            multi.addData("Step", lastCommand.label + " - " + lastCommand.status);
            if (!lastCommand.event.isEmpty()) multi.addData("EVENT", lastCommand.event);
            multi.addData("Shots fed", "%d (%s)", runner.shotsFired(), shotLog.isEmpty() ? "none yet" : shotLog.get(shotLog.size() - 1));
            multi.addData("Drive cmd (x, y)", "%.2f, %.2f x %.2f power", lastCommand.x, lastCommand.y, drivePower.v());
        }
        if (hold != null) {
            multi.addData("Heading hold", hold.faulted() ? "FAULTED, driving without it: " + hold.faultReason()
                    : String.format(Locale.US, "ON  error %.1f deg, correction %.2f", hold.lastErrorDeg(), lastHoldYaw));
        } else if (headingHold.v()) {
            multi.addData("Heading hold", imuStatus);
        }
        multi.addData("Flywheel rpm", "%.0f (%s)", lastRpm, lastCommand.flywheelOn ? "ON" : "off");
        if (detector != null) {
            multi.addData("Ready", detector.ready() + (detector.noSignal() ? "  *** NO ENCODER SIGNAL: using timer ***" : "")
                    + (Double.isNaN(detector.baselineRpm()) ? "" : String.format(Locale.US, "  baseline %.0f rpm", detector.baselineRpm())));
        }
        if (battery != null) multi.addData("Battery", "%.2f V", battery.getVoltage());
        for (int i = 0; i < shotLog.size(); i++) multi.addData("Shot " + (i + 1), shotLog.get(i));
        if (finished && runner != null) {
            multi.addData("Calibration", String.format(Locale.US,
                    "full-power seconds driven: forward %.2f, strafe %.2f. Measure distance, type MEASURED_*_IN, tick APPLY_MEASURED_SPEEDS, press OPTIONS.",
                    runner.forwardEquivalentSeconds(), runner.strafeEquivalentSeconds()));
            if (!saveStatus.isEmpty()) multi.addData("Last save", saveStatus);
        }
        multi.addData("Loop", "%.0f ms", lastDt * 1000.0);
        multi.addData("Dashboard", dashStatus + " | " + dash.editSummary());
        dash.pushIfDirty();
        multi.update();
    }

    private static String routeName(int r) {
        return r >= 0 && r < ROUTE_NAMES.length ? ROUTE_NAMES[r] : "INVALID (" + r + ")";
    }

    private static String stepIcon(SimpleAutoPlan.Step s) {
        switch (s.kind) {
            case WAIT: return "WAIT";
            case DRIVE: return "DRIVE";
            default: return "SHOOT";
        }
    }

    private static String describe(SimpleAutoPlan.Step s) {
        switch (s.kind) {
            case SHOOT:
                return s.label + ": " + s.shots + " balls";
            case DRIVE:
                String dir = s.y > 0 ? "forward" : s.y < 0 ? "back" : s.x > 0 ? "strafe right" : "strafe left";
                return String.format(Locale.US, "%s: %s %.2f s%s", s.label, dir, s.seconds, s.intake ? " (intake on)" : "");
            default:
                return String.format(Locale.US, "%s: %.1f s%s", s.label, s.seconds, s.intake ? " (intake on)" : "");
        }
    }
}
