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
    public static final class Settings {
        public String flywheel="",hood="",compression="",intake="",transfer="",feeder="";
        public String entrySensor="",stageSensor="",exitSensor="",limelight="";
        public String huskyLens="";
        public boolean flywheelReverse,intakeReverse,transferReverse,feederReverse;
        public double flywheelTicksPerRev=Double.NaN, maxRpm=Double.NaN, mechanismPowerCap=Double.NaN;
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
        public double autoParkAt=Double.NaN,autoPositionTolerance=Double.NaN,autoHeadingTolerance=Double.NaN;
        public double autoPositionKp=Double.NaN,autoHeadingKp=Double.NaN,autoPowerCap=Double.NaN;
        public double startX=Double.NaN,startY=Double.NaN,startHeading=Double.NaN;
        public List<BallType> confirmedPreload=Collections.emptyList();
        public boolean benchAccepted;
        public void validate() {
            if(!benchAccepted)throw new IllegalStateException("Physical bench acceptance required");
            String[] names={flywheel,hood,compression,intake,transfer,feeder,entrySensor,stageSensor,exitSensor,limelight,huskyLens,
                    PedroConstants.FRONT_LEFT_MOTOR,PedroConstants.BACK_LEFT_MOTOR,PedroConstants.FRONT_RIGHT_MOTOR,
                    PedroConstants.BACK_RIGHT_MOTOR,PedroConstants.PINPOINT_NAME,TurretHardwareConfig.MOTOR_NAME,TurretHardwareConfig.ENCODER_NAME};
            Set<String> unique=new HashSet<String>();
            for(String n:names)if(n==null||n.trim().isEmpty()||!unique.add(n))throw new IllegalStateException("Unique measured hardware names required");
            for(double n:new double[]{flywheelTicksPerRev,maxRpm,mechanismPowerCap,intakePower,feedPower,reversePower,sensorDebounceSec,yawAtFullPower,aimTolerance,maxMotion,maxTurretRate,hiveSettleSec,hiveMaxAgeSec,scoringPositionTolerance,scoringHeadingTolerance})if(!Double.isFinite(n)||n<=0)throw new IllegalStateException("Measured speeds/tolerances required");
            if(intakePower>mechanismPowerCap||feedPower>mechanismPowerCap||reversePower>mechanismPowerCap)throw new IllegalStateException("Mechanism powers exceed bench cap");
            if(mechanismPowerCap>1||flywheelPidf==null||turretProfile==null||!turretProfile.measured
                    ||scoringTiming==null||footprint==null||floor==null||!floor.isConfigured()||pieces==null
                    ||tipModel==null||travel==null||shots==null||!shots.hasMeasuredCycle()||camera==null||cellClassifier==null
                    ||entryClassifier==null||cameraPipeline<0||allianceCellIds.length==0||recipes.isEmpty())throw new IllegalStateException("Measured mechanism/vision/planning configuration incomplete");
            if(!pieces.isTimingMeasured()||pieces.alliance==AllianceColor.UNKNOWN||pieces.pollenCameraId==pieces.redNectarCameraId
                    ||pieces.pollenCameraId==pieces.blueNectarCameraId||pieces.redNectarCameraId==pieces.blueNectarCameraId)
                throw new IllegalStateException("Measured floor camera timing, taught IDs and alliance required");
            for(double n:new double[]{pieces.associationGateInches,pieces.trackTimeoutSec,pieces.matureHitCount})
                if(!Double.isFinite(n)||n<=0)throw new IllegalStateException("Measured tracker thresholds required");
            for(ShotRecipe r:recipes)if(r==null||r.rpm>maxRpm||!r.rangeCalibrated)throw new IllegalStateException("Recipe requires characterized flywheel/target range");
            for(ShotRecipe r:recipes)if(r.settleSec>=scoringTiming.prepare||r.rpmStableSec>=scoringTiming.prepare)throw new IllegalStateException("Shot settling exceeds prepare timeout");
            if(hiveSettleSec>=scoringTiming.hive)throw new IllegalStateException("HIVE settling exceeds reacquisition timeout");
            Set<Integer> ids=new HashSet<Integer>();
            for(int id:allianceCellIds)if(id<0||!ids.add(id))throw new IllegalStateException("Unique measured target IDs required");
            for(double n:new double[]{flywheelPidf.p,flywheelPidf.i,flywheelPidf.d,flywheelPidf.f})if(!Double.isFinite(n)||n<0)throw new IllegalStateException("Measured flywheel PIDF required");
        }
        public FixedRouteAuto autonomous() {
            if(!Double.isFinite(startX)||!Double.isFinite(startY)||!Double.isFinite(startHeading))throw new IllegalStateException("Measured auto starting pose required");
            return new FixedRouteAuto(route,park,footprint,30,autoParkAt,autoPositionTolerance,
                    autoHeadingTolerance,autoPositionKp,autoHeadingKp,autoPowerCap);
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
