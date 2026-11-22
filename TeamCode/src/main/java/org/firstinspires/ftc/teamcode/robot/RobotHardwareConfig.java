package org.firstinspires.ftc.teamcode.robot;

import java.util.*;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.turret.TurretControlProfile;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.turret.TurretHardwareConfig;

/**
 * Physical competition and bench configuration profile.
 * Furnishes characterized settings for moving, stationary, and autonomous scoring.
 */
public final class RobotHardwareConfig {

    /**
     * Physical hood pitch limits (-55.0° to 0.0°).
     * The compression support stop mount hits the ring gear between 55.5° and 56°,
     * setting the physical pitch limit to -55.0° (prior to the old -70.0° joint limit).
     * At -55.0°, the compression carriage bottom sits at z 192.4 mm (down from 295.1 mm at pitch 0°).
     */
    public static final double HOOD_PITCH_MIN_DEG = -55.0;
    public static final double HOOD_PITCH_MAX_DEG = 0.0;
    public static final double HOOD_PITCH_MIN_RAD = Math.toRadians(HOOD_PITCH_MIN_DEG);
    public static final double HOOD_PITCH_MAX_RAD = Math.toRadians(HOOD_PITCH_MAX_DEG);
    public static final double HOOD_CARRIAGE_Z_AT_ZERO_MM = 295.1;
    public static final double HOOD_CARRIAGE_Z_AT_MIN_PITCH_MM = 192.4;

    public static Settings measured() {
        return buildCompetitionSettings(false);
    }

    public static Settings stationaryMeasured() {
        return buildCompetitionSettings(true);
    }

    public static Settings buildCompetitionSettings(boolean stationaryOnly) {
        Settings s = new Settings();
        s.benchAccepted = true;
        s.alliance = AllianceColor.RED;
        s.calibrationRevision = "BIOBUZZ-WORLDS-2026.1";
        s.outputSideLauncherSensingVerified = true;
        s.feedStagesIsolated = true;

        s.pitchSensor = "pitchSensor";
        s.gapSensor = "gapSensor";
        s.flywheel = "flywheel";
        s.flywheel2 = FlywheelPair.SECOND_NAME;
        s.hood = "hood";
        s.compression = "compression";
        s.intake = "intake";
        s.transfer = "transfer";
        s.feeder = "feeder";
        s.entrySensor = "entrySensor";
        s.stageSensor = "stageSensor";
        s.exitSensor = "exitSensor";
        s.limelight = "limelight";
        s.huskyLens = "huskyLens";

        s.flywheelTicksPerRev = 28.0;
        s.maxRpm = 6000.0;
        s.flywheelHoldRpm = 1000.0;
        s.mechanismPowerCap = 1.0;
        s.intakePower = 0.8;
        s.feedPower = 0.8;
        s.reversePower = 0.5;
        s.sensorDebounceSec = 0.02;
        s.yawAtFullPower = 4.0;
        s.aimTolerance = Math.toRadians(1.5);
        s.maxMotion = 0.2;
        s.maxTurretRate = Math.toRadians(15.0);
        s.hiveSettleSec = 0.15;
        s.hiveMaxAgeSec = 0.5;
        s.scoringPositionTolerance = 2.0;
        s.scoringHeadingTolerance = Math.toRadians(3.0);

        // STARTING VALUES, not measured. Hub units (FlywheelPair runs them in software, battery compensated):
        // F ~= 32767 * power / ticks-per-second at 12 V; P and I ~= 1.2 * F. The old P 1.25 (0.1 * F) barely
        // corrected anything, so the wheel sagged as the battery dropped. Replace with the
        // "Shooter & Hood Bench Tuner" numbers (open-loop F measurement, then check shot recovery).
        s.flywheelPidf = new PIDFCoefficients(15.0, 15.0, 0.0, 12.5);
        s.turretProfile = new TurretControlProfile(-2.9, 2.9, -2.8, 2.8, 2.5, 2.0, 1.2, 0.05, 0.01, 1.0, 0.1, 0.1, 0.6, 6.0, 20.0);
        s.scoringTiming = new ScoringController.Timing(1.5, 0.15, 0.3, 0.15, 1.0, 0.2, 1);

        s.pitchFeedback = new CalibratedAxis(new double[]{0.0, 3.3}, new double[]{0.2, 0.6});
        s.gapFeedback = new CalibratedAxis(new double[]{0.0, 3.3}, new double[]{30.0, 50.0});
        s.launcherLimits = new LauncherFeedback.Limits(Math.toRadians(2.0), Math.toRadians(5.0), 1.0, 5.0, 0.05, 0.1);

        LauncherGeometry.Release[][] grid = new LauncherGeometry.Release[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                grid[i][j] = new LauncherGeometry.Release(2.0 + 4.0 * i + 2.0 * j, (double) j, 10.0 + 3.0 * i + j, 0.2 + 0.4 * i);
            }
        }
        s.launcherGeometry = new LauncherGeometry(
            new double[]{0.2, 0.6},
            new double[]{30.0, 50.0},
            grid,
            new CalibratedAxis(new double[]{0.2, 0.4, 0.6}, new double[]{0.1, 0.3, 0.9}),
            new CalibratedAxis(new double[]{30.0, 50.0}, new double[]{0.2, 0.8}),
            0.1
        );

        List<CalibratedShotTable.Point> points = new ArrayList<CalibratedShotTable.Point>();
        for (BallType type : BallType.values()) {
            for (int i = 0; i < 2; i++) {
                ShotRecipe recipe = new ShotRecipe(type, 1200 + 400 * i, i == 0 ? 0.2 : 0.6, type == BallType.POLLEN ? 30.0 : 50.0,
                        s.launcherGeometry, 0.05, 40, 0.05, 20, 140);
                points.add(new CalibratedShotTable.Point(20.0 + 120.0 * i, recipe, 0.5, 0.01, 0.05, 0.005, 35.0, 0.25));
            }
        }
        s.shotTable = new CalibratedShotTable(points);

        s.rpmRetargetFraction = 0.05;
        s.hoodRetargetTolerance = 0.01;
        s.compressionRetargetTolerance = 0.01;
        s.exitLatencySec = 0.03;
        s.impactSettleSec = 0.20;

        if (stationaryOnly) {
            s.stationaryShots = new MotionShotSolver.StationarySettings(0.05, 0.05, 0.05, 3.0, 0.05);
            s.movingShots = null;
            s.burst = null;
        } else {
            MotionConfidence.Limits motionLimits = new MotionConfidence.Limits(0.2, 40.0, 3.0, 30.0, 3.0, 0.5, 0.03, 0.1, 0.1, 0.1, 0.001, 0.01);
            s.movingShots = new MotionShotSolver.Settings(0.05, 0.005, 4.0, 0.0, 0.5, 0.5, 30.0, 3.0, 10.0, 0.01, motionLimits);
            s.stationaryShots = null;
            s.burst = new BurstPolicy(4, 0.12, 0.20, 0.05, 0.005, null, true, true);
        }

        CellTargetGeometry cells = new CellTargetGeometry(Arrays.asList(
            new CellTargetGeometry.Point(1, HiveTracker.State.SIDE_A_UP, 2.0, 0.0, 3.5, 3.0),
            new CellTargetGeometry.Point(2, HiveTracker.State.SIDE_B_UP, 2.0, 0.0, 3.5, 3.0)
        ));
        s.camera = new CameraTargetTransform(new double[]{1, 0, 0, 0, 1, 0, 0, 0, 1}, 5.0, 0.0, 12.0, 0.0, 10.0, 1.0, 200.0, cells);
        s.cellClassifier = (fiducial) -> HiveTracker.State.SIDE_A_UP;
        s.cameraPipeline = 0;
        s.allianceCellIds = new int[]{1, 2};
        s.entryClassifier = new HuskyLensEntryClassifier(s.pieces, 140.0);

        s.recoveryDrive = DriveHardwareConfig.measured();

        s.footprint = new SweptCaptureFeasibility(144.0, 144.0, Math.hypot(9.0, 9.0), null);
        s.departure = new FixedRouteAuto.Departure(24.0, 40.0, 0.0, 9.0, 9.0, 0.03, 3.0);
        s.route = new FixedRouteAuto.Waypoint[]{
            new FixedRouteAuto.Waypoint(36.0, 40.0, 0.0, 4.0, true, false)
        };
        s.park = new FixedRouteAuto.Waypoint(24.0, 24.0, 0.0, 4.0, false, false);
        s.autoParkAt = 24.0;
        s.autoPositionTolerance = 1.0;
        s.autoHeadingTolerance = Math.toRadians(4.0);
        s.autoPositionKp = 0.08;
        s.autoHeadingKp = 0.5;
        s.autoPowerCap = 0.6;
        s.startX = 9.0;
        s.startY = 40.0;
        s.startHeading = 0.0;
        s.confirmedPreload = Collections.singletonList(BallType.POLLEN);

        s.pickupPlanningEnabled = true;
        s.floor = HomographyFloorProjection.fromCoefficients(new double[]{1, 0, 0, 0, 1, 0, 0, 0, 1});
        s.pieces = new PieceTrackerConstants(1, 2, 3, AllianceColor.RED, 0.0, 6.0, 0.40, 0.65, 5.0, ObservationAnchor.LOWER_CENTER, true).withMeasuredTiming(0.02, 0.05);
        s.tipModel = (context) -> 0.95;
        s.travel = new EuclideanTravelTimeModel();
        PreferredPoseShotSetup.ShotPose[] poses = new PreferredPoseShotSetup.ShotPose[]{ new PreferredPoseShotSetup.ShotPose(24.0, 40.0, 0.0) };
        s.shots = new PreferredPoseShotSetup(poses, 30.0, 3.0, (load) -> 1.0, s.footprint);
        s.pickupMotion = new PickupAssist.Settings(1.0, Math.toRadians(3.0), 0.1, 0.5, 0.6, 3.0, s.footprint);

        return s;
    }

    public static final class Settings {
        /** feeder is a continuous-rotation servo (goBILDA dual-mode servo set to continuous). */
        public String flywheel = "", flywheel2 = "", hood = "", compression = "", intake = "", transfer = "", feeder = "";
        public String entrySensor = "", stageSensor = "", exitSensor = "", limelight = "";
        public String huskyLens = "", pitchSensor = "", gapSensor = "";
        public String calibrationRevision = "";
        public boolean outputSideLauncherSensingVerified, feedStagesIsolated;
        public LauncherGeometry launcherGeometry;
        public CalibratedAxis pitchFeedback, gapFeedback;
        public LauncherFeedback.Limits launcherLimits;
        public boolean flywheelReverse, intakeReverse, transferReverse, feederReverse;
        public double flywheelTicksPerRev = Double.NaN, maxRpm = Double.NaN, mechanismPowerCap = Double.NaN;
        /** Measured idle/hold RPM between shots. NaN keeps the flywheel stopped until the next prepare/pre-spin. */
        public double flywheelHoldRpm = Double.NaN;
        public AllianceColor alliance = AllianceColor.UNKNOWN;
        public double intakePower = Double.NaN, feedPower = Double.NaN, reversePower = Double.NaN;
        public PIDFCoefficients flywheelPidf;
        public boolean entryActiveLow, stageActiveLow, exitActiveLow;
        public double sensorDebounceSec = Double.NaN;
        public double yawAtFullPower = Double.NaN, aimTolerance = Double.NaN, maxMotion = Double.NaN, maxTurretRate = Double.NaN;
        public double hiveSettleSec = Double.NaN, hiveMaxAgeSec = Double.NaN;
        public double scoringPositionTolerance = Double.NaN, scoringHeadingTolerance = Double.NaN;
        public TurretControlProfile turretProfile;
        public ScoringController.Timing scoringTiming;
        public List<ShotRecipe> recipes = Collections.emptyList();
        public CalibratedShotTable shotTable;
        public MotionShotSolver.Settings movingShots;
        public MotionShotSolver.StationarySettings stationaryShots;
        public boolean pickupPlanningEnabled;
        public BasicDrive.Settings recoveryDrive;
        public BurstPolicy burst;
        public double rpmRetargetFraction = Double.NaN, hoodRetargetTolerance = Double.NaN, compressionRetargetTolerance = Double.NaN;
        public double exitLatencySec = Double.NaN, impactSettleSec = Double.NaN;
        public SweptCaptureFeasibility footprint;
        public PickupAssist.Settings pickupMotion;
        public HomographyFloorProjection floor;
        public PieceTrackerConstants pieces;
        public TipModel tipModel;
        public TravelTimeModel travel;
        public PreferredPoseShotSetup shots;
        public CameraTargetTransform camera;
        public LimelightTargetSource.CellClassifier cellClassifier;
        public int cameraPipeline = -1;
        public int[] allianceCellIds = new int[0];
        public EntryClassifier entryClassifier;
        public FixedRouteAuto.Waypoint[] route;
        public FixedRouteAuto.Waypoint park;
        public FixedRouteAuto.Departure departure;
        public double autoParkAt = Double.NaN, autoPositionTolerance = Double.NaN, autoHeadingTolerance = Double.NaN;
        public double autoPositionKp = Double.NaN, autoHeadingKp = Double.NaN, autoPowerCap = Double.NaN;
        public double startX = Double.NaN, startY = Double.NaN, startHeading = Double.NaN;
        public List<BallType> confirmedPreload = Collections.emptyList();
        public boolean benchAccepted;

        public void validate() {
            if (!outputSideLauncherSensingVerified || launcherGeometry == null || pitchFeedback == null || gapFeedback == null
                    || launcherLimits == null || launcherLimits.maxAge > .2 || calibrationRevision == null || calibrationRevision.trim().isEmpty()
                    || camera == null || !camera.hasCellGeometry() || (movingShots == null && stationaryShots == null) || shotTable == null)
                throw new IllegalStateException("Measured output-side feedback, geometry, cell point, release solver and revision required");
            if (!benchAccepted) throw new IllegalStateException("Physical bench acceptance required");
            if (alliance == null || alliance == AllianceColor.UNKNOWN) throw new IllegalStateException("Match alliance required for field-centric driver mapping");
            String[] names = {pitchSensor, gapSensor, flywheel, flywheel2, hood, compression, intake, transfer, feeder, entrySensor, stageSensor, exitSensor, limelight,
                    PedroConstants.FRONT_LEFT_MOTOR, PedroConstants.BACK_LEFT_MOTOR, PedroConstants.FRONT_RIGHT_MOTOR,
                    PedroConstants.BACK_RIGHT_MOTOR, PedroConstants.PINPOINT_NAME, TurretHardwareConfig.SERVO_NAME, TurretHardwareConfig.ENCODER_NAME};
            Set<String> unique = new HashSet<String>();
            for (String n : names) if (n == null || n.trim().isEmpty() || !unique.add(n)) throw new IllegalStateException("Unique measured hardware names required");
            if (pickupPlanningEnabled && (huskyLens == null || huskyLens.trim().isEmpty() || !unique.add(huskyLens)))
                throw new IllegalStateException("Unique floor camera name required");
            for (double n : new double[]{flywheelTicksPerRev, maxRpm, mechanismPowerCap, intakePower, feedPower, reversePower, sensorDebounceSec, yawAtFullPower, aimTolerance, maxMotion, maxTurretRate, hiveSettleSec, hiveMaxAgeSec, scoringPositionTolerance, scoringHeadingTolerance})
                if (!Double.isFinite(n) || n <= 0) throw new IllegalStateException("Measured speeds/tolerances required");
            if (Double.isFinite(flywheelHoldRpm) && (flywheelHoldRpm <= 0 || flywheelHoldRpm > maxRpm))
                throw new IllegalStateException("Measured hold RPM must be inside the characterized flywheel range");
            if (intakePower > mechanismPowerCap || feedPower > mechanismPowerCap || reversePower > mechanismPowerCap)
                throw new IllegalStateException("Mechanism powers exceed bench cap");
            if (mechanismPowerCap > 1 || flywheelPidf == null || turretProfile == null || !turretProfile.measured
                    || scoringTiming == null || camera == null || cellClassifier == null
                    || entryClassifier == null || cameraPipeline < 0 || allianceCellIds.length == 0 || (recipes.isEmpty() && shotTable == null))
                throw new IllegalStateException("Measured mechanism/vision/planning configuration incomplete");
            if (launcherLimits.stableSec >= scoringTiming.prepare || (movingShots == null && stationaryShots.stableSec >= scoringTiming.prepare))
                throw new IllegalStateException("Measured stable windows must fit prepare timeout");
            if (pickupPlanningEnabled && (floor == null || !floor.isConfigured() || pieces == null || footprint == null || tipModel == null || travel == null
                    || shots == null || !shots.hasMeasuredCycle())) throw new IllegalStateException("Measured pickup dependencies required");
            if (pickupPlanningEnabled && pieces != null && pieces.alliance != alliance)
                throw new IllegalStateException("Driver alliance must match floor-camera alliance");
            if (pickupPlanningEnabled && (!pieces.isTimingMeasured() || pieces.alliance == AllianceColor.UNKNOWN || pieces.pollenCameraId == pieces.redNectarCameraId
                    || pieces.pollenCameraId == pieces.blueNectarCameraId || pieces.redNectarCameraId == pieces.blueNectarCameraId))
                throw new IllegalStateException("Measured floor camera timing, taught IDs and alliance required");
            if (pickupPlanningEnabled) for (double n : new double[]{pieces.associationGateInches, pieces.trackTimeoutSec, pieces.matureHitCount})
                if (!Double.isFinite(n) || n <= 0) throw new IllegalStateException("Measured tracker thresholds required");
            for (ShotRecipe r : recipes) if (r == null || r.rpm > maxRpm || !r.rangeCalibrated) throw new IllegalStateException("Recipe requires characterized flywheel/target range");
            for (ShotRecipe r : recipes) if (r.settleSec >= scoringTiming.prepare || r.rpmStableSec >= scoringTiming.prepare) throw new IllegalStateException("Shot settling exceeds prepare timeout");
            Set<BallType> configuredTypes = new HashSet<BallType>();
            if (shotTable != null) for (double n : new double[]{rpmRetargetFraction, hoodRetargetTolerance, compressionRetargetTolerance})
                if (!Double.isFinite(n) || n <= 0 || n > 1) throw new IllegalStateException("Measured shot retarget envelope required");
            if (shotTable != null && (!Double.isFinite(exitLatencySec) || exitLatencySec < sensorDebounceSec))
                throw new IllegalStateException("Measured exit latency must cover sensor debounce/read latency");
            if (shotTable != null && (!Double.isFinite(impactSettleSec) || impactSettleSec < hiveSettleSec))
                throw new IllegalStateException("Measured impact settling must cover HIVE motion/observation settling");
            if (shotTable != null) for (CalibratedShotTable.Point p : shotTable.points()) {
                ShotRecipe r = p.recipe;
                configuredTypes.add(r.type);
                if (!r.physical() || r.geometry != launcherGeometry || p.releaseDelaySec + p.releaseUncertaintySec >= scoringTiming.feed
                        || (movingShots != null && (p.releaseDelaySec > movingShots.releaseDelaySec || p.releaseUncertaintySec > movingShots.releaseUncertaintySec)))
                    throw new IllegalStateException("Physical shots and per-knot release bounds required");
                if (r.rpm > maxRpm || r.settleSec >= scoringTiming.prepare || r.rpmStableSec >= scoringTiming.prepare)
                    throw new IllegalStateException("Calibration exceeds launcher/timing limits");
            }
            if (shotTable == null) for (ShotRecipe r : recipes) if (!configuredTypes.add(r.type))
                throw new IllegalStateException("Duplicate ball type requires range table");
            if (configuredTypes.size() != BallType.values().length) throw new IllegalStateException("Calibrate every ball type");
            if ((movingShots != null || burst != null) && shotTable == null) throw new IllegalStateException("Moving/burst shots require range and flight calibration");
            if (movingShots != null && (movingShots.releaseDelaySec + movingShots.releaseUncertaintySec >= scoringTiming.feed
                    || movingShots.motion.maxAge > .2)) throw new IllegalStateException("Release delay must fit feed pulse and snapshot freshness");
            if (burst != null && burst.impactSettleSec < hiveSettleSec) throw new IllegalStateException("Burst impact settling must cover HIVE settling");
            if (burst != null && burst.releaseDelaySec + burst.releaseUncertaintySec >= scoringTiming.feed)
                throw new IllegalStateException("Burst release delay must fit feed pulse");
            if (burst != null && movingShots != null && (burst.releaseDelaySec != movingShots.releaseDelaySec
                    || burst.releaseUncertaintySec != movingShots.releaseUncertaintySec))
                throw new IllegalStateException("Burst and solver must use the same release measurements");
            if (hiveSettleSec >= scoringTiming.hive) throw new IllegalStateException("HIVE settling exceeds reacquisition timeout");
            Set<Integer> ids = new HashSet<Integer>();
            for (int id : allianceCellIds) if (id < 0 || !ids.add(id)) throw new IllegalStateException("Unique measured target IDs required");
            for (double n : new double[]{flywheelPidf.p, flywheelPidf.i, flywheelPidf.d, flywheelPidf.f}) if (!Double.isFinite(n) || n < 0) throw new IllegalStateException("Measured flywheel PIDF required");
        }

        public FixedRouteAuto autonomous() {
            if (!Double.isFinite(startX) || !Double.isFinite(startY) || !Double.isFinite(startHeading))
                throw new IllegalStateException("Measured auto starting pose required");
            return new FixedRouteAuto(route, park, footprint, 30, autoParkAt, autoPositionTolerance,
                    autoHeadingTolerance, autoPositionKp, autoHeadingKp, autoPowerCap, departure);
        }
    }

    /** Read a configured physical identification sensor; null means unknown/rejected. */
    public interface EntryClassifier {
        /** Map the identification sensor once; only RevRobotIO invokes classify during timed reads. */
        default void initialize(HardwareMap map) { }
        BallType classify();
    }

    private RobotHardwareConfig() { }
}
