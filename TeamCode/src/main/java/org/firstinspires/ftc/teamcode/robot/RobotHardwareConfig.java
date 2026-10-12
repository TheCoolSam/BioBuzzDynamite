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

/** Physical configuration is undecided. Supply measured settings here after bench acceptance.
 * Missing settings keep both competition OpModes in an informative, powerless state.
 */
public final class RobotHardwareConfig {
    public static Settings measured() { return null; }
    public static Settings stationaryMeasured() {return null;}
    public static final class Settings {
        public String flywheel="",hood="",compression="",intake="",transfer="",feeder="";
        public String entrySensor="",stageSensor="",exitSensor="",limelight="";
        public String huskyLens="",pitchSensor="",gapSensor="";
        public String calibrationRevision="";
        public boolean outputSideLauncherSensingVerified,feedStagesIsolated;
        public LauncherGeometry launcherGeometry;
        public CalibratedAxis pitchFeedback,gapFeedback;
        public LauncherFeedback.Limits launcherLimits;
        public boolean flywheelReverse,intakeReverse,transferReverse,feederReverse;
        public double flywheelTicksPerRev=Double.NaN, maxRpm=Double.NaN, mechanismPowerCap=Double.NaN;
        /** Measured idle/hold RPM between shots. NaN keeps the flywheel stopped until the next prepare/pre-spin. */
        public double flywheelHoldRpm=Double.NaN;
        public AllianceColor alliance=AllianceColor.UNKNOWN;
        public double intakePower=Double.NaN,feedPower=Double.NaN,reversePower=Double.NaN;
        public PIDFCoefficients flywheelPidf;
        public boolean entryActiveLow,stageActiveLow,exitActiveLow;
        public double sensorDebounceSec=Double.NaN;
        public double yawAtFullPower=Double.NaN,aimTolerance=Double.NaN,maxMotion=Double.NaN,maxTurretRate=Double.NaN;
        public double hiveSettleSec=Double.NaN,hiveMaxAgeSec=Double.NaN;
        public double scoringPositionTolerance=Double.NaN,scoringHeadingTolerance=Double.NaN;
        public TurretControlProfile turretProfile;
        public ScoringController.Timing scoringTiming;
        public List<ShotRecipe> recipes=Collections.emptyList();
        public CalibratedShotTable shotTable;
        public MotionShotSolver.Settings movingShots;
        public MotionShotSolver.StationarySettings stationaryShots;
        public boolean pickupPlanningEnabled;
        public BasicDrive.Settings recoveryDrive;
        public BurstPolicy burst;
        public double rpmRetargetFraction=Double.NaN,hoodRetargetTolerance=Double.NaN,compressionRetargetTolerance=Double.NaN;
        public double exitLatencySec=Double.NaN,impactSettleSec=Double.NaN;
        public SweptCaptureFeasibility footprint;
        public PickupAssist.Settings pickupMotion;
        public HomographyFloorProjection floor;
        public PieceTrackerConstants pieces;
        public TipModel tipModel;
        public TravelTimeModel travel;
        public PreferredPoseShotSetup shots;
        public CameraTargetTransform camera;
        public LimelightTargetSource.CellClassifier cellClassifier;
        public int cameraPipeline=-1;
        public int[] allianceCellIds=new int[0];
        public EntryClassifier entryClassifier;
        public FixedRouteAuto.Waypoint[] route;
        public FixedRouteAuto.Waypoint park;
        public FixedRouteAuto.Departure departure;
        public double autoParkAt=Double.NaN,autoPositionTolerance=Double.NaN,autoHeadingTolerance=Double.NaN;
        public double autoPositionKp=Double.NaN,autoHeadingKp=Double.NaN,autoPowerCap=Double.NaN;
        public double startX=Double.NaN,startY=Double.NaN,startHeading=Double.NaN;
        public List<BallType> confirmedPreload=Collections.emptyList();
        public boolean benchAccepted;
        public void validate() {
            if(!outputSideLauncherSensingVerified||launcherGeometry==null||pitchFeedback==null||gapFeedback==null
                    ||launcherLimits==null||launcherLimits.maxAge>.2||calibrationRevision==null||calibrationRevision.trim().isEmpty()
                    ||camera==null||!camera.hasCellGeometry()||(movingShots==null&&stationaryShots==null)||shotTable==null)
                throw new IllegalStateException("Measured output-side feedback, geometry, cell point, release solver and revision required");
            if(!benchAccepted)throw new IllegalStateException("Physical bench acceptance required");
            if(alliance==null||alliance==AllianceColor.UNKNOWN)throw new IllegalStateException("Match alliance required for field-centric driver mapping");
            String[] names={pitchSensor,gapSensor,flywheel,hood,compression,intake,transfer,feeder,entrySensor,stageSensor,exitSensor,limelight,
                    PedroConstants.FRONT_LEFT_MOTOR,PedroConstants.BACK_LEFT_MOTOR,PedroConstants.FRONT_RIGHT_MOTOR,
                    PedroConstants.BACK_RIGHT_MOTOR,PedroConstants.PINPOINT_NAME,TurretHardwareConfig.SERVO_NAME,TurretHardwareConfig.ENCODER_NAME};
            Set<String> unique=new HashSet<String>();
            for(String n:names)if(n==null||n.trim().isEmpty()||!unique.add(n))throw new IllegalStateException("Unique measured hardware names required");
            if(pickupPlanningEnabled&&(huskyLens==null||huskyLens.trim().isEmpty()||!unique.add(huskyLens)))
                throw new IllegalStateException("Unique floor camera name required");
            for(double n:new double[]{flywheelTicksPerRev,maxRpm,mechanismPowerCap,intakePower,feedPower,reversePower,sensorDebounceSec,yawAtFullPower,aimTolerance,maxMotion,maxTurretRate,hiveSettleSec,hiveMaxAgeSec,scoringPositionTolerance,scoringHeadingTolerance})if(!Double.isFinite(n)||n<=0)throw new IllegalStateException("Measured speeds/tolerances required");
            if(Double.isFinite(flywheelHoldRpm)&&(flywheelHoldRpm<=0||flywheelHoldRpm>maxRpm))
                throw new IllegalStateException("Measured hold RPM must be inside the characterized flywheel range");
            if(intakePower>mechanismPowerCap||feedPower>mechanismPowerCap||reversePower>mechanismPowerCap)throw new IllegalStateException("Mechanism powers exceed bench cap");
            if(mechanismPowerCap>1||flywheelPidf==null||turretProfile==null||!turretProfile.measured
                    ||scoringTiming==null||camera==null||cellClassifier==null
                    ||entryClassifier==null||cameraPipeline<0||allianceCellIds.length==0||(recipes.isEmpty()&&shotTable==null))throw new IllegalStateException("Measured mechanism/vision/planning configuration incomplete");
            if(launcherLimits.stableSec>=scoringTiming.prepare||(movingShots==null&&stationaryShots.stableSec>=scoringTiming.prepare))
                throw new IllegalStateException("Measured stable windows must fit prepare timeout");
            if(pickupPlanningEnabled&&(floor==null||!floor.isConfigured()||pieces==null||footprint==null||tipModel==null||travel==null
                    ||shots==null||!shots.hasMeasuredCycle()))throw new IllegalStateException("Measured pickup dependencies required");
            if(pickupPlanningEnabled&&pieces!=null&&pieces.alliance!=alliance)
                throw new IllegalStateException("Driver alliance must match floor-camera alliance");
            if(pickupPlanningEnabled&&(!pieces.isTimingMeasured()||pieces.alliance==AllianceColor.UNKNOWN||pieces.pollenCameraId==pieces.redNectarCameraId)
                    ||pieces.pollenCameraId==pieces.blueNectarCameraId||pieces.redNectarCameraId==pieces.blueNectarCameraId))
                throw new IllegalStateException("Measured floor camera timing, taught IDs and alliance required");
            if(pickupPlanningEnabled)for(double n:new double[]{pieces.associationGateInches,pieces.trackTimeoutSec,pieces.matureHitCount})
                if(!Double.isFinite(n)||n<=0)throw new IllegalStateException("Measured tracker thresholds required");
            for(ShotRecipe r:recipes)if(r==null||r.rpm>maxRpm||!r.rangeCalibrated)throw new IllegalStateException("Recipe requires characterized flywheel/target range");
            for(ShotRecipe r:recipes)if(r.settleSec>=scoringTiming.prepare||r.rpmStableSec>=scoringTiming.prepare)throw new IllegalStateException("Shot settling exceeds prepare timeout");
            Set<BallType> configuredTypes=new HashSet<BallType>();
            if(shotTable!=null)for(double n:new double[]{rpmRetargetFraction,hoodRetargetTolerance,compressionRetargetTolerance})
                if(!Double.isFinite(n)||n<=0||n>1)throw new IllegalStateException("Measured shot retarget envelope required");
            if(shotTable!=null&&(!Double.isFinite(exitLatencySec)||exitLatencySec<sensorDebounceSec))
                throw new IllegalStateException("Measured exit latency must cover sensor debounce/read latency");
            if(shotTable!=null&&(!Double.isFinite(impactSettleSec)||impactSettleSec<hiveSettleSec))
                throw new IllegalStateException("Measured impact settling must cover HIVE motion/observation settling");
            if(shotTable!=null)for(CalibratedShotTable.Point p:shotTable.points()) {
                ShotRecipe r=p.recipe;configuredTypes.add(r.type);
                if(!r.physical()||r.geometry!=launcherGeometry||p.releaseDelaySec+p.releaseUncertaintySec>=scoringTiming.feed
                        ||(movingShots!=null&&(p.releaseDelaySec>movingShots.releaseDelaySec||p.releaseUncertaintySec>movingShots.releaseUncertaintySec)))
                    throw new IllegalStateException("Physical shots and per-knot release bounds required");
                if(r.rpm>maxRpm||r.settleSec>=scoringTiming.prepare||r.rpmStableSec>=scoringTiming.prepare)
                    throw new IllegalStateException("Calibration exceeds launcher/timing limits");
            }
            if(shotTable==null)for(ShotRecipe r:recipes)if(!configuredTypes.add(r.type))
                throw new IllegalStateException("Duplicate ball type requires range table");
            if(configuredTypes.size()!=BallType.values().length)throw new IllegalStateException("Calibrate every ball type");
            if((movingShots!=null||burst!=null)&&shotTable==null)throw new IllegalStateException("Moving/burst shots require range and flight calibration");
            if(movingShots!=null&&(movingShots.releaseDelaySec+movingShots.releaseUncertaintySec>=scoringTiming.feed
                    ||movingShots.motion.maxAge>.2))throw new IllegalStateException("Release delay must fit feed pulse and snapshot freshness");
            if(burst!=null&&burst.impactSettleSec<hiveSettleSec)throw new IllegalStateException("Burst impact settling must cover HIVE settling");
            if(burst!=null&&burst.releaseDelaySec+burst.releaseUncertaintySec>=scoringTiming.feed)
                throw new IllegalStateException("Burst release delay must fit feed pulse");
            if(burst!=null&&movingShots!=null&&(burst.releaseDelaySec!=movingShots.releaseDelaySec
                    ||burst.releaseUncertaintySec!=movingShots.releaseUncertaintySec))
                throw new IllegalStateException("Burst and solver must use the same release measurements");
            if(hiveSettleSec>=scoringTiming.hive)throw new IllegalStateException("HIVE settling exceeds reacquisition timeout");
            Set<Integer> ids=new HashSet<Integer>();
            for(int id:allianceCellIds)if(id<0||!ids.add(id))throw new IllegalStateException("Unique measured target IDs required");
            for(double n:new double[]{flywheelPidf.p,flywheelPidf.i,flywheelPidf.d,flywheelPidf.f})if(!Double.isFinite(n)||n<0)throw new IllegalStateException("Measured flywheel PIDF required");
        }
        public FixedRouteAuto autonomous() {
            if(!Double.isFinite(startX)||!Double.isFinite(startY)||!Double.isFinite(startHeading))throw new IllegalStateException("Measured auto starting pose required");
            return new FixedRouteAuto(route,park,footprint,30,autoParkAt,autoPositionTolerance,
                    autoHeadingTolerance,autoPositionKp,autoHeadingKp,autoPowerCap,departure);
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
