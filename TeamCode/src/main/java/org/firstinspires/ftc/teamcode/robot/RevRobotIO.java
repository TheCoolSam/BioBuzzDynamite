package org.firstinspires.ftc.teamcode.robot;

import java.util.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.pedropathing.follower.Follower;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.scoring.*;

/** Configured SDK implementation with final independent output/freshness interlocks. */
public final class RevRobotIO implements RobotIO {
    private final RobotHardwareConfig.Settings settings;
    private Follower follower;
    private PedroRobotStateSource source;
    private final RobotStateEstimator estimator=new RobotStateEstimator();
    private final RobotStateHistory history=new RobotStateHistory();
    private final TurretPoseHistory turretHistory=new TurretPoseHistory();
    private GoBildaPinpointDriver pinpoint;
    private RevTurretIO turret;
    private DcMotorEx flywheel;
    private DcMotor intake,transfer,feeder;
    private Servo hood,compression;
    private AnalogInput pitchSensor,gapSensor;
    private LauncherFeedback launcher;
    private double previousPitch=Double.NaN,previousGap=Double.NaN,previousLauncherSec=Double.NaN;
    private DigitalChannel entry,stage,exit;
    private BooleanDebouncer entryDebounce,stageDebounce,exitDebounce;
    private final List<DcMotor> motors=new ArrayList<DcMotor>();
    private LimelightTargetSource target;
    private long lastReadNanos;
    private boolean healthy,fault;
    private double lastDt;
    private HuskyLensWorker floor;
    private RevBasicDrive recoveryDrive;
    private boolean recovering;
    private final HubBulkCache bulk;
    public RevRobotIO(HardwareMap map,RobotHardwareConfig.Settings settings,long epochNanos) {
        if(settings==null)throw new IllegalStateException("Hardware configuration undecided");
        settings.validate();this.settings=settings;
        List<HubBulkCache.Hub> hubs=new ArrayList<HubBulkCache.Hub>();
        for(LynxModule module:map.getAll(LynxModule.class)) {
            final LynxModule hub=module;
            hubs.add(new HubBulkCache.Hub() {
                public void setManual() {hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);}
                public void clear() {hub.clearBulkCache();}
            });
        }
        if(hubs.isEmpty())throw new IllegalStateException("Lynx hubs required for MANUAL bulk caching");
        bulk=new HubBulkCache(hubs);bulk.enableManual();
        try {
            if(PedroConstants.LOCALIZER!=PedroConstants.LocalizerKind.PINPOINT)throw new IllegalStateException("Competition adapter requires verified Pinpoint health");
            for(String name:new String[]{PedroConstants.FRONT_LEFT_MOTOR,PedroConstants.BACK_LEFT_MOTOR,
                    PedroConstants.FRONT_RIGHT_MOTOR,PedroConstants.BACK_RIGHT_MOTOR})motor(map,name);
            follower=PedroFactory.create(map);follower.manual(DrivePowers.zero());source=new PedroRobotStateSource(follower);
            if(settings.recoveryDrive!=null) {
                String[] names={PedroConstants.FRONT_LEFT_MOTOR,PedroConstants.BACK_LEFT_MOTOR,PedroConstants.FRONT_RIGHT_MOTOR,PedroConstants.BACK_RIGHT_MOTOR};
                DcMotorSimple.Direction[] directions={PedroConstants.FRONT_LEFT_DIRECTION,PedroConstants.BACK_LEFT_DIRECTION,PedroConstants.FRONT_RIGHT_DIRECTION,PedroConstants.BACK_RIGHT_DIRECTION};
                for(int i=0;i<4;i++)if(!names[i].equals(settings.recoveryDrive.names[i])
                        ||settings.recoveryDrive.reversed[i]!=(directions[i]==DcMotorSimple.Direction.REVERSE))
                    throw new IllegalStateException("Recovery drive must match verified competition motor names/signs");
                recoveryDrive=new RevBasicDrive(map,settings.recoveryDrive,bulk);
            }
            pinpoint=map.get(GoBildaPinpointDriver.class,PedroConstants.PINPOINT_NAME);
            turret=new RevTurretIO(map,settings.turretProfile);
            flywheel=map.get(DcMotorEx.class,settings.flywheel);flywheel.setPower(0);motors.add(flywheel);
            flywheel.setDirection(settings.flywheelReverse?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,settings.flywheelPidf);
            intake=motor(map,settings.intake);transfer=motor(map,settings.transfer);feeder=motor(map,settings.feeder);
            intake.setDirection(settings.intakeReverse?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            transfer.setDirection(settings.transferReverse?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            feeder.setDirection(settings.feederReverse?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            hood=map.get(Servo.class,settings.hood);compression=map.get(Servo.class,settings.compression);
            pitchSensor=map.get(AnalogInput.class,settings.pitchSensor);gapSensor=map.get(AnalogInput.class,settings.gapSensor);
            entry=sensor(map,settings.entrySensor);stage=sensor(map,settings.stageSensor);exit=sensor(map,settings.exitSensor);
            entryDebounce=new BooleanDebouncer(settings.sensorDebounceSec);
            stageDebounce=new BooleanDebouncer(settings.sensorDebounceSec);exitDebounce=new BooleanDebouncer(settings.sensorDebounceSec);
            settings.entryClassifier.initialize(map);
            target=new LimelightTargetSource(map.get(Limelight3A.class,settings.limelight),settings.camera,
                    settings.cameraPipeline,settings.allianceCellIds,settings.cellClassifier);target.start();
            if(settings.pickupPlanningEnabled) {
                HuskyLensPieceObservationSource pieces=new HuskyLensPieceObservationSource(map.get(HuskyLens.class,settings.huskyLens));
                if(!pieces.knock()||!pieces.selectColorRecognition())throw new IllegalStateException("HuskyLens not ready");
                floor=new HuskyLensWorker(pieces,epochNanos);floor.start();
            }
            stop();
        } catch(RuntimeException failure) {close();throw failure;}
    }
    private DcMotor motor(HardwareMap map,String name) {
        DcMotor motor=map.get(DcMotor.class,name);motor.setPower(0);motors.add(motor);
        return motor;
    }
    private static DigitalChannel sensor(HardwareMap map,String name) {
        DigitalChannel sensor=map.get(DigitalChannel.class,name);sensor.setMode(DigitalChannel.Mode.INPUT);return sensor;
    }
    public void setStartingPose(double x,double y,double heading) {
        stop();follower.setPose(new Pose(x,y,heading));source.localizationReset();history.clear();turretHistory.clear();target.reset();
    }
    @Override public Snapshot read(double now,double dt) {
        if(recovering)throw new IllegalStateException("Recovery mode owns drive outputs");
        bulk.beginLoop();
        lastReadNanos=System.nanoTime();
        if(fault)throw new IllegalStateException("Latched hardware fault; reinitialize OpMode");
        if(!TurretEncoderSample.validDt(dt)) {stop();healthy=false;}
        try {follower.update();}catch(RuntimeException failure){fault=true;stop();throw failure;}
        boolean odometryHealthy=pinpoint.getDeviceStatus()==GoBildaPinpointDriver.DeviceStatus.READY;
        source.capture(now,odometryHealthy);
        RobotState robot=estimator.update(source,now);
        turret.read(dt);
        double pitch=Double.NaN,gap=Double.NaN;
        try {
            double pitchVoltage=pitchSensor.getVoltage(),gapVoltage=gapSensor.getVoltage();
            pitch=pitchVoltage>=0&&pitchVoltage<=pitchSensor.getMaxVoltage()?settings.pitchFeedback.map(pitchVoltage):Double.NaN;
            gap=gapVoltage>=0&&gapVoltage<=gapSensor.getMaxVoltage()?settings.gapFeedback.map(gapVoltage):Double.NaN;
        }catch(RuntimeException sensorFailure) { /* Launcher health fails; no sensor value is invented. */ }
        double elapsed=now-previousLauncherSec;
        boolean derivative=Double.isFinite(elapsed)&&elapsed>0&&elapsed<=settings.launcherLimits.maxAge;
        launcher=new LauncherFeedback(now,pitch,derivative?(pitch-previousPitch)/elapsed:Double.NaN,
                gap,derivative?(gap-previousGap)/elapsed:Double.NaN);
        previousPitch=pitch;previousGap=gap;previousLauncherSec=now;
        double rpm=Double.NaN;
        try {rpm=flywheel.getVelocity()*60/settings.flywheelTicksPerRev;}catch(RuntimeException sensorFailure) { }
        history.add(robot);
        if(turret.sample.valid)turretHistory.add(now,turret.getAngleRad(),robot.getResetGeneration());
        TargetEstimate aim=null;
        try {aim=target.read(now,history,turretHistory,robot.getResetGeneration());}catch(RuntimeException visionFailure) {target.reset();}
        lastDt=dt;
        boolean entryBlocked=false,staged=false,exited=false,inventoryHealthy=true;
        org.firstinspires.ftc.teamcode.planning.pickup.BallType type=null;
        try {
            entryBlocked=entryDebounce.update(entry.getState()!=settings.entryActiveLow,now);
            staged=stageDebounce.update(stage.getState()!=settings.stageActiveLow,now);
            exited=exitDebounce.update(exit.getState()!=settings.exitActiveLow,now);
            type=entryBlocked?settings.entryClassifier.classify():null;
        }catch(RuntimeException sensorFailure) {inventoryHealthy=false;}
        boolean timing=TurretEncoderSample.validDt(dt)&&(System.nanoTime()-lastReadNanos)*1e-9<=.2;
        Health health=new Health(!fault&&timing,robot.isFresh(now,.2),turret.sample.valid,
                Double.isFinite(rpm)&&Double.isFinite(pitch)&&Double.isFinite(gap),inventoryHealthy);
        healthy=health.allHealthy();
        return new Snapshot(robot,now,turret.getAngleRad(),turret.getVelocityRadPerSec(),rpm,health,
                entryBlocked,staged,exited,type,aim,floor==null?Collections.emptyList():floor.latest(),launcher);
    }
    @Override public void write(Command command) {
        double age=(System.nanoTime()-lastReadNanos)*1e-9;
        if(recovering||command==null||!command.enabled||fault||!healthy||age<0||age>.2
                ||!TurretEncoderSample.validDt(lastDt)) {stop();return;}
        for(double n:new double[]{command.fieldX,command.fieldY,command.turn,command.turretPower,
                command.rpm,command.intake,command.transfer,command.feeder}) {
            if(!Double.isFinite(n)) {fault=true;stop();return;}
        }
        if(command.rpm<0||command.rpm>settings.maxRpm
                ||(!Double.isNaN(command.hood)&&(command.hood<0||command.hood>1||!Double.isFinite(command.hood)))
                ||(!Double.isNaN(command.compression)&&(command.compression<0||command.compression>1||!Double.isFinite(command.compression)))) {
            fault=true;stop();return;
        }
        if(Double.isFinite(command.hood)||Double.isFinite(command.compression)||command.feeder>0) {
            double hoodCommand=settings.launcherGeometry.pitchServo.map(command.pitchRad);
            double gapCommand=settings.launcherGeometry.gapServo.map(command.gapMm);
            if(!Double.isFinite(hoodCommand)||!Double.isFinite(gapCommand)
                    ||Math.abs(command.hood-hoodCommand)>1e-8||Math.abs(command.compression-gapCommand)>1e-8) {
                fault=true;stop();throw new IllegalStateException("Launcher commands outside calibrated physical map");
            }
            if(command.feeder>0&&(!launcher.fresh(command.acquiredSec+age,settings.launcherLimits.maxAge)
                    ||!Double.isFinite(launcher.pitchRad)||!Double.isFinite(launcher.pitchVelocity)
                    ||!Double.isFinite(launcher.gapMm)||!Double.isFinite(launcher.gapVelocity)
                    ||Math.abs(launcher.pitchRad-command.pitchRad)>settings.launcherLimits.pitchError
                    ||Math.abs(launcher.pitchVelocity)>settings.launcherLimits.pitchRate
                    ||Math.abs(launcher.gapMm-command.gapMm)>settings.launcherLimits.gapError
                    ||Math.abs(launcher.gapVelocity)>settings.launcherLimits.gapRate)) {
                fault=true;stop();throw new IllegalStateException("Final measured launcher feed interlock");
            }
        }
        try {
            if(!PedroManualDrive.request(follower,clamp(command.fieldX,1),clamp(command.fieldY,1),clamp(command.turn,1)))throw new IllegalStateException("Drive request refused");
            turret.command(command.turretPower,true,true,false);
            if(!turret.motorFault.isEmpty())throw new IllegalStateException("Turret write fault");
            flywheel.setVelocity(command.rpm*settings.flywheelTicksPerRev/60);
            intake.setPower(clamp(command.intake,settings.mechanismPowerCap));
            transfer.setPower(clamp(command.transfer,settings.mechanismPowerCap));
            feeder.setPower(clamp(command.feeder,settings.mechanismPowerCap));
            if(Double.isFinite(command.hood))hood.setPosition(command.hood);
            if(Double.isFinite(command.compression))compression.setPosition(command.compression);
        } catch(RuntimeException failure) {fault=true;stop();throw failure;}
    }
    private static double clamp(double power,double cap) {return Math.max(-cap,Math.min(cap,power));}
    @Override public boolean enterDriveRecovery() {
        stop();
        if(fault||recoveryDrive==null||!recoveryDrive.healthy())return false;
        if(floor!=null)floor.close();
        recovering=true;return true;
    }
    @Override public Snapshot readDriveRecovery(double now,double dt) {
        bulk.beginLoop();
        lastReadNanos=System.nanoTime();lastDt=dt;
        return new Snapshot(RobotState.invalid(now),now,Double.NaN,Double.NaN,Double.NaN,
                new Health(recovering&&!fault&&recoveryDrive!=null&&recoveryDrive.healthy()&&TurretEncoderSample.validDt(dt),false,false,false,false),
                false,false,false,null,null,Collections.emptyList(),null);
    }
    @Override public void writeDriveRecovery(Command command,double dt) {
        double age=(System.nanoTime()-lastReadNanos)*1e-9;
        if(!recovering||fault||recoveryDrive==null||command==null||!command.enabled||age<0||age>.2
                ||command.rpm!=0||command.turretPower!=0||command.intake!=0||command.transfer!=0||command.feeder!=0
                ||!Double.isNaN(command.hood)||!Double.isNaN(command.compression)
                ||!Double.isNaN(command.pitchRad)||!Double.isNaN(command.gapMm)) {stop();return;}
        if(!recoveryDrive.write(command.fieldX,command.fieldY,command.turn,true,dt)) {fault=true;stop();}
    }
    @Override public void stop() {
        // Attempt every independent actuator even if an earlier write failed.
        if(follower!=null)try{follower.manual(DrivePowers.zero());}catch(RuntimeException e){fault=true;}
        for(DcMotor motor:motors)try{motor.setPower(0);}catch(RuntimeException e){fault=true;}
        if(turret!=null) {turret.stop();if(!turret.motorFault.isEmpty())fault=true;}
        if(recoveryDrive!=null&&!recoveryDrive.stop())fault=true;
    }
    public void close() {stop();if(floor!=null)floor.close();if(target!=null)target.stop();}
}
