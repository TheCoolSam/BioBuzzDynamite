package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.state.PoseStorage;
import com.pedropathing.math.Pose;
import java.util.*;

/**
 * Integrated calibrated scoring TeleOp.
 * Streamlined single-driver competition controls with auto-prespin, auto-burst, and hands-off pickup assist.
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class BioBuzzTeleOp extends OpMode {
    public static final int MAX_INVENTORY_CAPACITY = 4;
    public static final double SLOW_MODE_TRIGGER_THRESHOLD = 0.2;
    public static final double SLOW_MODE_SCALE = 0.45;
    public static final double INTAKE_TRIGGER_THRESHOLD = 0.2;
    public static final double BURST_HOLD_DURATION_SEC = 0.25;

    protected RobotHardwareConfig.Settings settings;
    protected RevRobotIO io;
    protected RobotCoordinator robot;
    protected long epochNanos, lastNanos;
    protected String configurationFault = "Hardware measurements are undecided";
    private double lastLog;
    private PickupAssist pickup;
    private boolean previousPickupHold;
    private String pickupReason = "";
    private final List<BallType> recoveryDraft = new ArrayList<BallType>();
    private boolean previousPollen, previousNectar, previousUndo, previousClear, previousCommit;
    private boolean previousHeadingReset;
    private double shootPressStartTime = Double.NaN;
    private final LoopMetrics metrics = new LoopMetrics();
    private final LoopMetrics intervals = new LoopMetrics();
    private boolean previousHit, previousMiss, previousTip, previousNoTip;
    private int previousPieceCount = 0;
    private double lastAimRumbleTime = 0;

    protected RobotHardwareConfig.Settings loadSettings() {
        return RobotHardwareConfig.measured();
    }

    protected boolean stationaryProfileRequired() {
        return false;
    }

    @Override
    public void init() {
        epochNanos = lastNanos = System.nanoTime();
        telemetry.setMsTransmissionInterval(100);
        settings = loadSettings();
        if (settings != null) {
            try {
                if (stationaryProfileRequired() && (settings.stationaryShots == null || settings.movingShots != null || settings.burst != null)) {
                    throw new IllegalStateException("Stationary profile requires stationary bounds and no moving/burst mode");
                }
                settings.validate();
                io = new RevRobotIO(hardwareMap, settings, epochNanos);
                PieceInventory inventory = new PieceInventory(MAX_INVENTORY_CAPACITY);
                List<BallType> preload = settings.confirmedPreload;
                if (PoseStorage.hasPose() && !PoseStorage.getInventory().isEmpty()) {
                    preload = PoseStorage.getInventory();
                }
                inventory.reconcile(preload);
                ScoringController scoring = settings.shotTable == null
                        ? new ScoringController(inventory, settings.recipes, settings.scoringTiming, settings.feedPower, settings.reversePower)
                        : new ScoringController(inventory, settings.shotTable, settings.scoringTiming, settings.feedPower, settings.reversePower, settings.burst);
                if (settings.shotTable != null) {
                    scoring.setRetargetTolerance(settings.rpmRetargetFraction, settings.hoodRetargetTolerance, settings.compressionRetargetTolerance);
                    scoring.setFlightTiming(settings.exitLatencySec, settings.impactSettleSec);
                }
                if (Double.isFinite(settings.flywheelHoldRpm)) {
                    scoring.setHoldRpm(settings.flywheelHoldRpm);
                }
                robot = new RobotCoordinator(io, inventory, scoring,
                        settings.pickupPlanningEnabled ? new PieceTracker(settings.floor, settings.pieces) : null,
                        settings.pickupPlanningEnabled ? new PickupPlanner(settings.tipModel, settings.travel, settings.shots, settings.footprint) : null,
                        new TurretController(settings.turretProfile), new DriveAssist(settings.yawAtFullPower, 0.3, 0.08),
                        new HiveTracker(settings.hiveSettleSec, settings.hiveMaxAgeSec), settings.aimTolerance, settings.maxMotion, settings.maxTurretRate);
                robot.setShotLogging(settings.calibrationRevision, Long.toHexString(System.currentTimeMillis()) + "-" + Long.toHexString(epochNanos));
                robot.setShotTipModel(settings.tipModel);
                RobotLog.ii("BioBuzzShot", "%s", ShotJournal.HEADER);
                scoring.setLauncherLimits(settings.launcherLimits);
                robot.setFeedStagesIsolated(settings.feedStagesIsolated);
                robot.setIntakePower(settings.intakePower);
                robot.setTurretPivot(settings.camera.pivotForward(), settings.camera.pivotLeft());
                robot.setTargetMaxAgeSec(settings.hiveMaxAgeSec);
                if (settings.shotTable == null) {
                    robot.setScoringPoses(settings.shots, settings.scoringPositionTolerance, settings.scoringHeadingTolerance);
                }
                if (settings.movingShots != null) {
                    robot.setMovingShots(settings.movingShots);
                } else {
                    robot.setStationaryShots(settings.stationaryShots);
                }
                configurationFault = "";
            } catch (RuntimeException failure) {
                configurationFault = failure.getMessage();
                if (io != null) io.close();
                io = null;
                robot = null;
            }
        }
        display();
    }

    @Override
    public void init_loop() {
        if (io != null) {
            try {
                long n = System.nanoTime();
                io.read(seconds(n), (n - lastNanos) * 1e-9);
                lastNanos = n;
                io.stop();
            } catch (RuntimeException failure) {
                configurationFault = failure.getMessage();
                io.close();
                io = null;
                robot = null;
            }
        }
        display();
    }

    @Override
    public void start() {
        lastNanos = System.nanoTime();
        if (io != null) {
            io.stop();
            if (PoseStorage.hasPose()) {
                Pose autoPose = PoseStorage.getPose();
                io.setStartingPose(autoPose.x(), autoPose.y(), autoPose.heading());
            }
        }
    }

    protected double seconds(long nanos) {
        return (nanos - epochNanos) * 1e-9;
    }

    protected RobotCoordinator.Input input() {
        RobotCoordinator.Input input = new RobotCoordinator.Input();
        double now = seconds(System.nanoTime());

        // Match Enable: Active by default; Circle (B) is emergency stop / cancel.
        input.cancel = gamepad1.b;
        input.enabled = !input.cancel;

        // Drive Sticks via DriverStickMap (alliance-aware)
        DriverStickMap.Command sticks = (robot != null && robot.driveRecovery())
                ? DriverStickMap.robotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
                : DriverStickMap.fieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,
                        settings == null ? AllianceColor.UNKNOWN : settings.alliance);

        // L2 (Left Trigger): Precision slow-speed crawl mode (45% power)
        double speedScale = gamepad1.left_trigger > SLOW_MODE_TRIGGER_THRESHOLD ? SLOW_MODE_SCALE : 1.0;
        input.fieldX = sticks.fieldX * speedScale;
        input.fieldY = sticks.fieldY * speedScale;
        input.turn = sticks.turn * speedScale;

        boolean neutral = sticks.fieldX == 0 && sticks.fieldY == 0 && sticks.turn == 0;

        // Reset Latched Fault: Neutral sticks with Back+Start or Circle
        input.resetFault = ((gamepad1.back && gamepad1.start) || (gamepad1.b && neutral)) && neutral;

        // Emergency Drive Recovery: Back + Y with neutral sticks
        input.recoverDrive = gamepad1.back && gamepad1.y && neutral && !input.enabled;

        // Field Heading Reset: Triangle (Y) with neutral sticks
        boolean headingReset = gamepad1.y && neutral && !gamepad1.back;
        if (headingReset && !previousHeadingReset && io != null && robot != null && robot.snapshot != null && robot.snapshot.robot != null) {
            io.setStartingPose(robot.snapshot.robot.getFieldX(), robot.snapshot.robot.getFieldY(), 0.0);
        }
        previousHeadingReset = headingReset;

        // Shooting: R1 (or A). Tap = single shot, Hold (>= 0.25s) = validated burst
        boolean shootPressed = gamepad1.right_bumper || gamepad1.a;
        if (shootPressed) {
            if (Double.isNaN(shootPressStartTime)) {
                shootPressStartTime = now;
            }
            input.shoot = true;
            if (now - shootPressStartTime >= BURST_HOLD_DURATION_SEC && settings != null && settings.burst != null) {
                input.burst = true;
            }
        } else {
            shootPressStartTime = Double.NaN;
        }

        // Auto Pre-Spin: Flywheel automatically pre-spins when holding pieces in inventory, or manual G2 LB
        boolean holdingPieces = robot != null && robot.inventory != null && !robot.inventory.pieces().isEmpty();
        input.preSpin = holdingPieces || gamepad2.left_bumper;

        // Hands-off Auto-Pickup Assist: L1 (Left Bumper) or Square (X)
        boolean holdPickup = gamepad1.left_bumper || gamepad1.x;

        // Intake: R2 (Right Trigger) or manual LT. Auto-stops when inventory is full
        boolean full = robot != null && robot.inventory != null && robot.inventory.pieces().size() >= MAX_INVENTORY_CAPACITY;
        input.intake = (gamepad1.right_trigger > INTAKE_TRIGGER_THRESHOLD || gamepad1.left_trigger > 0.5) && !full;

        // Smart auto-unwind when idle or moving without an active shot, plus manual D-pad right
        boolean shouldAutoUnwind = robot != null && robot.snapshot != null
                && Math.abs(robot.snapshot.turretAngle) > 2.0
                && !input.shoot && !input.intake && !holdPickup
                && robot.scoring.state() == ScoringController.State.IDLE;
        input.unwindAssist = gamepad1.dpad_right || shouldAutoUnwind;
        if (!holdPickup || !input.enabled || input.cancel || input.shoot || robot == null
                || !robot.inventory.isConfirmed() || robot.driveRecovery()
                || robot.scoring.state() != ScoringController.State.IDLE) {
            pickup = null;
        } else if (holdPickup && !previousPickupHold && settings != null && settings.pickupMotion != null && robot.snapshot != null) {
            try {
                pickup = new PickupAssist(settings.pickupMotion, robot.plan, robot.snapshot.robot,
                        now, robot.inventory.pieces().size());
            } catch (IllegalArgumentException refused) {
                pickupReason = refused.getMessage();
            }
        }
        previousPickupHold = holdPickup;

        if (pickup != null) {
            PickupAssist.Command move = pickup.update(robot.snapshot.robot, now, robot.inventory.pieces().size());
            input.fieldX = move.x;
            input.fieldY = move.y;
            input.turn = move.turn;
            input.intake = move.intake;
            input.expectedPickupType = pickup.expectedType;
            input.unwindAssist = false;
            pickupReason = move.reason;
        }

        return input;
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) * 1e-9;
        lastNanos = now;
        intervals.record(dt);

        if (robot != null) {
            RobotCoordinator.Input command = input();
            recoverInventory(command);
            robot.cycle(command, seconds(now), dt);
            annotateOutcome(seconds(now));

            // Gamepad 1 Haptic Feedback
            if (robot.inventory != null) {
                int currentCount = robot.inventory.pieces().size();
                if (currentCount > previousPieceCount) {
                    if (currentCount >= MAX_INVENTORY_CAPACITY) {
                        gamepad1.rumbleBlips(2);
                    } else {
                        gamepad1.rumbleBlips(1);
                    }
                }
                previousPieceCount = currentCount;
            }

            if (robot.scoring != null) {
                boolean aimLocked = robot.scoring.pitchReady && robot.scoring.compressionReady
                        && robot.scoring.rpmReady && robot.shotSolution != null && robot.shotSolution.confident;
                if (aimLocked && seconds(now) - lastAimRumbleTime >= 0.6) {
                    gamepad1.rumble(0.25, 0.25, 120);
                    lastAimRumbleTime = seconds(now);
                }
            }
        }

        if (robot != null && robot.journal != null) {
            String record;
            int count = 0;
            while (count++ < 8 && (record = robot.journal.poll()) != null) {
                RobotLog.ii("BioBuzzShot", "%s", record);
            }
        }

        if (robot != null && seconds(now) - lastLog >= 0.5) {
            double[] timing = metrics.summary();
            double[] cadence = intervals.summary();
            RobotLog.ii("BioBuzz", "t=%.3f scoring=%s inventory=%s inhibit=%s loopP50=%.4f loopP95=%.4f loopP99=%.4f max=%.4f intervalP99=%.4f intervalMax=%.4f read=%.4f plan=%.4f write=%.4f cycle=%.4f",
                    seconds(now), robot.scoring.state(), robot.inventory.load(), robot.reason, timing[0], timing[1], timing[2], timing[3],
                    cadence[2], cadence[3], robot.readSeconds, robot.planSeconds, robot.writeSeconds, robot.cycleSeconds);
            lastLog = seconds(now);
        }

        display();
        metrics.record((System.nanoTime() - now) * 1e-9);
    }

    private void annotateOutcome(double now) {
        boolean hit = gamepad2.a, miss = gamepad2.b, tip = gamepad2.left_stick_button, noTip = gamepad2.right_stick_button;
        if (robot.journal != null) {
            long id = robot.journal.lastConfirmedShotId();
            if (id > 0) {
                if (hit && !miss && !previousHit) robot.journal.outcome(id, now, "HIT", null);
                if (miss && !hit && !previousMiss) robot.journal.outcome(id, now, "MISS", null);
                if (tip && !noTip && !previousTip) robot.journal.outcome(id, now, null, "TIP");
                if (noTip && !tip && !previousNoTip) robot.journal.outcome(id, now, null, "NO_TIP");
            }
        }
        previousHit = hit;
        previousMiss = miss;
        previousTip = tip;
        previousNoTip = noTip;
    }

    private void recoverInventory(RobotCoordinator.Input command) {
        boolean pollen = gamepad2.dpad_up, nectar = gamepad2.dpad_down, undo = gamepad2.x, clear = gamepad2.y;
        boolean commit = gamepad2.back && gamepad2.start;
        if (!command.enabled && !command.shoot && !command.intake) {
            io.stop();
            if (clear && !previousClear) recoveryDraft.clear();
            if (undo && !previousUndo && !recoveryDraft.isEmpty()) recoveryDraft.remove(recoveryDraft.size() - 1);
            if (pollen && !previousPollen && recoveryDraft.size() < MAX_INVENTORY_CAPACITY) recoveryDraft.add(BallType.POLLEN);
            if (nectar && !previousNectar && recoveryDraft.size() < MAX_INVENTORY_CAPACITY) recoveryDraft.add(BallType.NECTAR);
            if (commit && !previousCommit) robot.inventory.reconcile(recoveryDraft);
        }
        previousPollen = pollen;
        previousNectar = nectar;
        previousUndo = undo;
        previousClear = clear;
        previousCommit = commit;
    }

    protected void display() {
        if (robot == null) {
            telemetry.addData("MOTION DISABLED", configurationFault);
            telemetry.addLine("Configure RobotHardwareConfig with measured hardware/recipes and bench acceptance.");
            return;
        }

        telemetry.addData("Auto Pose Handoff", PoseStorage.hasPose() ? "ACTIVE" : "STANDBY");
        telemetry.addData("Scoring", robot.scoring.state());
        telemetry.addData("Inventory", robot.inventory.load());
        telemetry.addData("Subsystem Health", robot.snapshot == null ? "AWAITING SAMPLE" : robot.snapshot.health);
        telemetry.addData("Drive Recovery", robot.driveRecovery() ? "ACTIVE (ROBOT-CENTRIC)" : "STANDBY");
        telemetry.addData("Inventory Confirmed", robot.inventory.isConfirmed());
        telemetry.addData("Balls In Flight", robot.scoring.inFlight().size());
        telemetry.addData("Shot Confidence", robot.shotConfidence);

        if (robot.shotSolution != null) {
            telemetry.addData("Equivalent Range (in)", String.format(Locale.US, "%.1f", robot.shotSolution.equivalentRange));
            telemetry.addData("Miss Bound (in)", String.format(Locale.US, "%.2f", robot.shotMissBoundInches));
        }

        telemetry.addData("Pitch Ready", robot.scoring.pitchReady);
        telemetry.addData("Gap Ready", robot.scoring.compressionReady);
        telemetry.addData("RPM Ready", robot.scoring.rpmReady);
        telemetry.addData("Inhibit Reason", robot.reason);

        telemetry.addLine("\n--- SINGLE DRIVER CONTROLS (PS5) ---");
        telemetry.addLine("L-Stick: Drive | R-Stick: Turn | L2: Precision Slow Mode");
        telemetry.addLine("R1: Shoot (Tap = 1, Hold = Burst) | R2: Intake (Auto-Stops when full)");
        telemetry.addLine("L1 / Square: Hold for Auto-Pickup Assist");
        telemetry.addLine("Circle: E-Stop / Cancel | Triangle (Stopped): Reset Heading");
    }

    @Override
    public void stop() {
        if (io != null) io.close();
    }
}
