package org.firstinspires.ftc.teamcode.robot;

import java.util.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** One snapshot, one actuator owner, coherent resets, and bounded lower-cadence planning. */
public final class RobotCoordinator {
    public static final class Input {
        public boolean enabled,intake,shoot,cancel,cancelShot,resetFault,unwindAssist;
        public double fieldX,fieldY,turn;
        public BallType expectedPickupType;
    }
    private final RobotIO io;
    public final PieceInventory inventory;
    public final ScoringController scoring;
    public final RobotStateHistory history=new RobotStateHistory();
    private final PieceTracker tracker;
    private final PickupPlanner planner;
    private final TurretController turret;
    private final DriveAssist assist;
    private final HiveTracker hive;
    private final double aimTolerance,maxMotion,maxTurretRate;
    private double intakePower=.25;
    private double pivotForward,pivotLeft;
    private int targetId=Integer.MIN_VALUE;
    private double hiveFrameFloor=Double.NEGATIVE_INFINITY;
    private PreferredPoseShotSetup scoringPoses;
    private double scoringPositionTolerance,scoringHeadingTolerance;
    private long generation=Long.MIN_VALUE,inventoryRevision;
    private double lastPlan=Double.NEGATIVE_INFINITY;
    private boolean entryPrevious,initialized,fault;
    public PickupPlan plan;
    public RobotIO.Snapshot snapshot;
    public String reason="INITIALIZING";
    public RobotCoordinator(RobotIO io,PieceInventory inventory,ScoringController scoring,PieceTracker tracker,
            PickupPlanner planner,TurretController turret,DriveAssist assist,HiveTracker hive,
            double aimTolerance,double maxMotion,double maxTurretRate) {
        if(io==null||inventory==null||scoring==null||tracker==null||planner==null||turret==null||assist==null||hive==null)throw new IllegalArgumentException("Configured subsystems required");
        for(double n:new double[]{aimTolerance,maxMotion,maxTurretRate})if(!Double.isFinite(n)||n<=0)throw new IllegalArgumentException("Measured firing tolerances required");
        this.io=io;this.inventory=inventory;this.scoring=scoring;this.tracker=tracker;this.planner=planner;
        this.turret=turret;this.assist=assist;this.hive=hive;this.aimTolerance=aimTolerance;this.maxMotion=maxMotion;this.maxTurretRate=maxTurretRate;
    }
    public void stop() {io.stop();scoring.cancel();plan=null;}
    public void setIntakePower(double measuredPower) {
        if(!Double.isFinite(measuredPower)||measuredPower<=0||measuredPower>1)throw new IllegalArgumentException("Measured intake power required");
        intakePower=measuredPower;
    }
    public void setTurretPivot(double forward,double left) {
        if(!Double.isFinite(forward)||!Double.isFinite(left))throw new IllegalArgumentException("Measured turret pivot required");
        pivotForward=forward;pivotLeft=left;
    }
    public void setScoringPoses(PreferredPoseShotSetup poses,double positionTolerance,double headingTolerance) {
        if(poses==null||!poses.hasMeasuredCycle()||!Double.isFinite(positionTolerance)||positionTolerance<=0
                ||!Double.isFinite(headingTolerance)||headingTolerance<=0)throw new IllegalArgumentException("Measured scoring poses/tolerances required");
        scoringPoses=poses;scoringPositionTolerance=positionTolerance;scoringHeadingTolerance=headingTolerance;
    }
    private RobotState pivotState(RobotState r) {
        double h=r.getHeadingRad(),x=Math.cos(h)*pivotForward-Math.sin(h)*pivotLeft;
        double y=Math.sin(h)*pivotForward+Math.cos(h)*pivotLeft,w=r.getAngularVelocityRadPerSec();
        return new RobotState(r.getTimestampSec(),r.getFieldX()+x,r.getFieldY()+y,h,
                r.getFieldVx()-w*y,r.getFieldVy()+w*x,w,0,0,0,r.isPoseValid(),r.isVelocityValid(),
                r.getAcquisitionTimestampSec(),r.isDeviceHealthy(),r.getResetGeneration());
    }
    public void localizationReset(double now,long generation) {
        io.stop();history.clear();tracker.reset(now,generation);turret.reset();hive.reset();plan=null;
        this.generation=generation;lastPlan=Double.NEGATIVE_INFINITY;scoring.cancel();targetId=Integer.MIN_VALUE;hiveFrameFloor=now;
    }
    public void consumedTrack(int id,double now) {tracker.consume(id,now);plan=null;}
    public void cycle(Input input,double now,double dt) {
        // Cancelling/releasing enable stops before any SDK reads can delay the write.
        if(!input.enabled||input.cancel)io.stop();
        RobotIO.Command command=new RobotIO.Command();
        try {
            snapshot=io.read(now,dt);
            if(snapshot==null||snapshot.robot==null||!snapshot.robot.isFresh(now,.2)||!snapshot.healthy
                    ||!TurretEncoderSample.validDt(dt)||!Double.isFinite(snapshot.acquiredSec)
                    ||now<snapshot.acquiredSec||now-snapshot.acquiredSec>.2) {
                fault=true;reason="SENSOR / TIMING FAULT";scoring.cancel();io.stop();return;
            }
            if(generation!=Long.MIN_VALUE&&snapshot.robot.getResetGeneration()<generation) {
                fault=true;reason="OLD LOCALIZATION GENERATION";scoring.cancel();io.stop();return;
            }
            if(generation!=snapshot.robot.getResetGeneration())localizationReset(now,snapshot.robot.getResetGeneration());
            history.add(snapshot.robot);
            if(fault) {
                if(input.resetFault&&!input.enabled&&!input.shoot&&!input.intake&&input.fieldX==0&&input.fieldY==0&&input.turn==0&&inventory.isConfirmed())fault=false;
                reason="FAULT: RELEASE ENABLE AND RESET";io.stop();return;
            }
            boolean entered=initialized&&snapshot.entry&&!entryPrevious;
            entryPrevious=snapshot.entry;initialized=true;
            if(entered) {
                if(input.expectedPickupType!=null&&snapshot.entryType!=input.expectedPickupType) {
                    inventory.markUncertain();fault=true;reason="PICKUP TYPE MISMATCH; RECONCILE";io.stop();return;
                }
                long reservation=input.intake&&input.enabled?inventory.reserve(snapshot.entryType):-1;
                if(reservation<0||!inventory.confirmEntry(reservation)) {
                    inventory.markUncertain();fault=true;reason="UNEXPECTED / UNKNOWN PIECE ENTRY";io.stop();return;
                }
                // Intake confirmation clears the strategic commitment even if counts later match.
                plan=null;
                if(tracker.size()>0) { tracker.reset(now,generation); }
            }
            PieceTrackingResult tracked=tracker.update(snapshot.pieces,history,now);
            if(inventoryRevision!=inventory.revision()) {plan=null;inventoryRevision=inventory.revision();}
            if(input.cancel)plan=null;
            if(now-lastPlan>=.1) {
                plan=planner.plan(snapshot.robot,inventory.load(),tracked.getPieces(),now,plan);lastPlan=now;
            }
            boolean targetFresh=snapshot.target!=null&&snapshot.target.fresh(now,.2,generation);
            if(targetFresh&&targetId!=snapshot.target.targetId) {hive.reset();turret.reset();targetId=snapshot.target.targetId;}
            if(targetFresh&&snapshot.target.exposureSec>hiveFrameFloor)hive.observe(snapshot.target.hive,snapshot.target.exposureSec,true);
            RobotState pivot=pivotState(snapshot.robot);
            TurretCommand aim=targetFresh?turret.calculate(TurretStateAdapter.toTurretState(pivot,
                    snapshot.turretAngle,snapshot.turretVelocity,snapshot.target.x,snapshot.target.y,dt,now)):null;
            if(!targetFresh)turret.reset();
            ScoringController.Input shot=new ScoringController.Input();
            shot.nowSec=now;shot.acquiredSec=snapshot.acquiredSec;shot.healthy=snapshot.healthy;
            shot.enabled=input.enabled;shot.start=input.shoot;shot.cancel=input.cancel||input.cancelShot;
            shot.resetFault=input.resetFault&&!input.enabled&&!input.shoot&&!input.intake&&input.fieldX==0&&input.fieldY==0&&input.turn==0;
            shot.staged=snapshot.staged;shot.exit=snapshot.exit;shot.rpm=snapshot.rpm;
            shot.targetFresh=targetFresh;shot.targetObservedSec=targetFresh?snapshot.target.exposureSec:Double.NaN;
            shot.hiveObservedSec=shot.targetObservedSec;
            shot.rangeInches=targetFresh?Math.hypot(snapshot.target.x-pivot.getFieldX(),snapshot.target.y-pivot.getFieldY()):Double.NaN;
            shot.aimReady=aim!=null&&aim.tracking&&aim.targetReachable
                    &&Math.abs(AngleUtil.wrapRadians(aim.targetBearing-snapshot.robot.getHeadingRad()-snapshot.turretAngle))<=aimTolerance;
            if(scoringPoses!=null) {
                ShotSetupPlan setup=scoringPoses.select(snapshot.robot.getFieldX(),snapshot.robot.getFieldY(),snapshot.robot.getHeadingRad(),inventory.load());
                shot.aimReady &= setup!=null&&Math.hypot(setup.x-snapshot.robot.getFieldX(),setup.y-snapshot.robot.getFieldY())<=scoringPositionTolerance
                        &&Math.abs(AngleUtil.wrapRadians(setup.headingRad-snapshot.robot.getHeadingRad()))<=scoringHeadingTolerance;
            }
            shot.stationary=snapshot.robot.isVelocityValid()&&Math.hypot(snapshot.robot.getFieldVx(),snapshot.robot.getFieldVy())<=maxMotion
                    &&Math.abs(snapshot.robot.getAngularVelocityRadPerSec())<=maxTurretRate&&Math.abs(snapshot.turretVelocity)<=maxTurretRate;
            HiveTracker.State hiveState=hive.state(now);
            shot.hiveStable=hiveState==HiveTracker.State.SIDE_A_UP||hiveState==HiveTracker.State.SIDE_B_UP;
            ScoringController.State before=scoring.state();
            ScoringController.Output output=scoring.update(shot);
            if(before!=ScoringController.State.WAIT_FOR_HIVE&&scoring.state()==ScoringController.State.WAIT_FOR_HIVE) {
                hive.reset();hiveFrameFloor=now;
            }
            if(input.enabled&&!input.cancel) {
                command.enabled=true;command.fieldX=input.fieldX;command.fieldY=input.fieldY;
                command.turn=assist.turn(input.turn,aim==null?0:aim.requestedChassisOmega,input.unwindAssist,true);
                command.turretPower=aim==null?0:aim.turretMotorPower;
                command.rpm=output.rpm;command.hood=output.hood;command.compression=output.compression;
                command.transfer=output.transfer;command.feeder=output.feeder;
                command.intake=input.intake&&inventory.canIntake()&&scoring.state()==ScoringController.State.IDLE?intakePower:0;
            }
            command.reason=reason=output.reason;io.write(command);
        } catch(RuntimeException failure) {fault=true;reason="ROBOT FAULT: "+failure.getClass().getSimpleName();scoring.cancel();io.stop();}
    }
}
