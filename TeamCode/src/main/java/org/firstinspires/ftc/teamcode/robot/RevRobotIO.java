package org.firstinspires.ftc.teamcode.robot;

import java.util.*;
import com.qualcomm.robotcore.hardware.*;
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
    private DigitalChannel entry,stage,exit;
    private BooleanDebouncer entryDebounce,stageDebounce,exitDebounce;
    private final List<DcMotor> motors=new ArrayList<DcMotor>();
    private LimelightTargetSource target;
    private long lastReadNanos;
    private boolean healthy,fault;
    private double lastDt;
    private HuskyLensWorker floor;
    public RevRobotIO(HardwareMap map,RobotHardwareConfig.Settings settings,long epochNanos) {
        if(settings==null)throw new IllegalStateException("Hardware configuration undecided");
        settings.validate();this.settings=settings;
        try {
            if(PedroConstants.LOCALIZER!=PedroConstants.LocalizerKind.PINPOINT)throw new IllegalStateException("Competition adapter requires verified Pinpoint health");
            for(String name:new String[]{PedroConstants.FRONT_LEFT_MOTOR,PedroConstants.BACK_LEFT_MOTOR,
                    PedroConstants.FRONT_RIGHT_MOTOR,PedroConstants.BACK_RIGHT_MOTOR})motor(map,name);
            follower=PedroFactory.create(map);follower.manual(DrivePowers.zero());source=new PedroRobotStateSource(follower);
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
            entry=sensor(map,settings.entrySensor);stage=sensor(map,settings.stageSensor);exit=sensor(map,settings.exitSensor);
            entryDebounce=new BooleanDebouncer(settings.sensorDebounceSec);
            stageDebounce=new BooleanDebouncer(settings.sensorDebounceSec);exitDebounce=new BooleanDebouncer(settings.sensorDebounceSec);
            settings.entryClassifier.initialize(map);
            target=new LimelightTargetSource(map.get(Limelight3A.class,settings.limelight),settings.camera,
                    settings.cameraPipeline,settings.allianceCellIds,settings.cellClassifier);target.start();
            HuskyLensPieceObservationSource pieces=new HuskyLensPieceObservationSource(map.get(HuskyLens.class,settings.huskyLens));
            if(!pieces.knock()||!pieces.selectColorRecognition())throw new IllegalStateException("HuskyLens not ready");
            floor=new HuskyLensWorker(pieces,epochNanos);floor.start();
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
        lastReadNanos=System.nanoTime();
        if(fault)throw new IllegalStateException("Latched hardware fault; reinitialize OpMode");
        if(!TurretEncoderSample.validDt(dt)) {stop();healthy=false;}
        follower.update();
        boolean odometryHealthy=pinpoint.getDeviceStatus()==GoBildaPinpointDriver.DeviceStatus.READY;
        source.capture(now,odometryHealthy);
        RobotState robot=estimator.update(source,now);
        turret.read(dt);
        double rpm=flywheel.getVelocity()*60/settings.flywheelTicksPerRev;
        healthy=robot.isFresh(now,.2)&&turret.sample.valid&&Double.isFinite(rpm)&&TurretEncoderSample.validDt(dt);
        history.add(robot);
        if(turret.sample.valid)turretHistory.add(now,turret.getAngleRad(),robot.getResetGeneration());
        TargetEstimate aim=target.read(now,history,turretHistory,robot.getResetGeneration());
        lastDt=dt;
        boolean entryBlocked=entryDebounce.update(entry.getState()!=settings.entryActiveLow,now);
        boolean staged=stageDebounce.update(stage.getState()!=settings.stageActiveLow,now);
        boolean exited=exitDebounce.update(exit.getState()!=settings.exitActiveLow,now);
        org.firstinspires.ftc.teamcode.planning.pickup.BallType type=entryBlocked?settings.entryClassifier.classify():null;
        healthy &= (System.nanoTime()-lastReadNanos)*1e-9<=.2;
        return new Snapshot(robot,now,turret.getAngleRad(),turret.getVelocityRadPerSec(),rpm,healthy,
                entryBlocked,staged,exited,type,aim,floor.latest());
    }
    @Override public void write(Command command) {
        double age=(System.nanoTime()-lastReadNanos)*1e-9;
        if(command==null||!command.enabled||fault||!healthy||age<0||age>.2
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
    @Override public void stop() {
        // Attempt every independent actuator even if an earlier write failed.
        if(follower!=null)try{follower.manual(DrivePowers.zero());}catch(RuntimeException e){fault=true;}
        for(DcMotor motor:motors)try{motor.setPower(0);}catch(RuntimeException e){fault=true;}
        if(turret!=null) {turret.stop();if(!turret.motorFault.isEmpty())fault=true;}
    }
    public void close() {stop();if(floor!=null)floor.close();if(target!=null)target.stop();}
}
