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
        public long shotRequestId;
        public boolean enabled,intake,shoot,burst,preSpin,cancel,cancelShot,resetFault,unwindAssist,recoverDrive,mechanismsInhibited;
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
    private boolean feedStagesIsolated;
    public ShotJournal journal;
    public double readSeconds,planSeconds,writeSeconds,cycleSeconds;
    private TipModel shotTipModel;
    public double shotTipProbability=Double.NaN;
    public void setShotLogging(String revision) {journal=new ShotJournal(revision);}
    public void setShotLogging(String revision,String runId) {journal=new ShotJournal(revision,runId);}
    public void setShotTipModel(TipModel model) {shotTipModel=model;}
    public void setFeedStagesIsolated(boolean measuredIsolation) {feedStagesIsolated=measuredIsolation;}
    private double pivotForward,pivotLeft;
    private MotionShotSolver movingShots;
    private MotionShotSolver.StationarySettings stationaryShots;
    private double stationarySince=Double.NaN;
    public void setStationaryShots(MotionShotSolver.StationarySettings settings) {stationaryShots=settings;}
    private MotionConfidence motionConfidence;
    private double committedReleaseSec=Double.NaN;
    private double previousTurretAngle=Double.NaN,previousTurretRate,previousTurretSec;
    public MotionShotSolver.Solution shotSolution;
    public double shotMissBoundInches=Double.NaN;
    public String shotConfidence="STATIONARY SHOTS";
    private int targetId=Integer.MIN_VALUE;
    private double hiveFrameFloor=Double.NEGATIVE_INFINITY;
    private PreferredPoseShotSetup scoringPoses;
    private double scoringPositionTolerance,scoringHeadingTolerance;
    private long generation=Long.MIN_VALUE,inventoryRevision;
    private double lastPlan=Double.NEGATIVE_INFINITY;
    private boolean entryPrevious,initialized,fault;
    private boolean driveRecovery;
    private final HealthWatch healthWatch=new HealthWatch();
    public HealthWatch.View healthView;
    public boolean driveRecovery() {return driveRecovery;}
    public PickupPlan plan;
    public RobotIO.Snapshot snapshot;
    public String reason="INITIALIZING";
    public RobotCoordinator(RobotIO io,PieceInventory inventory,ScoringController scoring,PieceTracker tracker,
            PickupPlanner planner,TurretController turret,DriveAssist assist,HiveTracker hive,
            double aimTolerance,double maxMotion,double maxTurretRate) {
        if(io==null||inventory==null||scoring==null||(tracker==null)!=(planner==null)||turret==null||assist==null||hive==null)throw new IllegalArgumentException("Configured subsystems required");
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
    public void setMovingShots(MotionShotSolver.Settings settings) {
        movingShots=new MotionShotSolver(settings);motionConfidence=new MotionConfidence(settings.motion);
        committedReleaseSec=Double.NaN;
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
        io.stop();history.clear();if(tracker!=null)tracker.reset(now,generation);turret.reset();hive.reset();plan=null;
        stationarySince=Double.NaN;
        this.generation=generation;lastPlan=Double.NEGATIVE_INFINITY;scoring.cancel();targetId=Integer.MIN_VALUE;hiveFrameFloor=now;
        if(motionConfidence!=null)motionConfidence.reset();committedReleaseSec=Double.NaN;shotSolution=null;
        previousTurretAngle=Double.NaN;
    }
    public void consumedTrack(int id,double now) {if(tracker!=null)tracker.consume(id,now);plan=null;}
    public void cycle(Input input,double now,double dt) {
        long cycleStarted=System.nanoTime();readSeconds=planSeconds=writeSeconds=0;
        // Cancelling/releasing enable stops before any SDK reads can delay the write.
        if(!input.enabled||input.cancel)io.stop();
        RobotIO.Command command=new RobotIO.Command();
        try {
            if(input.recoverDrive&&!input.enabled&&!input.shoot&&!input.intake&&!input.preSpin
                    &&input.shotRequestId==0&&input.fieldX==0&&input.fieldY==0&&input.turn==0
                    &&(fault||snapshot!=null&&!snapshot.healthy||healthView!=null&&healthView.anyLatched())) {
                io.stop();scoring.cancel();plan=null;
                if(io.enterDriveRecovery()) {driveRecovery=true;healthWatch.reset();history.clear();hive.reset();turret.reset();}
                else reason="DRIVE RECOVERY REFUSED: MEASURED DRIVE / CONFIRMED STOP REQUIRED";
            }
            if(driveRecovery) {
                long readStarted=System.nanoTime();
                try {snapshot=io.readDriveRecovery(now,dt);}finally {readSeconds=(System.nanoTime()-readStarted)*1e-9;}
                boolean fresh=snapshot!=null&&snapshot.health.drive&&Double.isFinite(now)&&Double.isFinite(snapshot.acquiredSec)
                        &&now>=snapshot.acquiredSec&&now-snapshot.acquiredSec<=.2&&TurretEncoderSample.validDt(dt);
                command.enabled=fresh&&input.enabled&&!input.cancel&&BasicDrive.powers(input.fieldX,input.fieldY,input.turn,1)!=null;
                if(command.enabled) {command.fieldX=input.fieldX;command.fieldY=input.fieldY;command.turn=input.turn;command.acquiredSec=now;}
                reason=command.reason="LATCHED ROBOT-CENTRIC DRIVE ONLY / MECHANISMS INHIBITED";
                long writeStarted=System.nanoTime();
                try {io.writeDriveRecovery(command,dt);}finally {writeSeconds=(System.nanoTime()-writeStarted)*1e-9;}return;
            }
            long readStarted=System.nanoTime();
            try {snapshot=io.read(now,dt);}finally {readSeconds=(System.nanoTime()-readStarted)*1e-9;}
            boolean timingOk=snapshot!=null&&snapshot.robot!=null&&snapshot.robot.isFresh(now,.2)
                    &&TurretEncoderSample.validDt(dt)&&Double.isFinite(snapshot.acquiredSec)
                    &&now>=snapshot.acquiredSec&&now-snapshot.acquiredSec<=.2;
            healthView=healthWatch.update(snapshot==null?null:snapshot.health,timingOk);
            if(healthView.inventoryLatched)inventory.markUncertain();
            boolean latch=healthView.driveLatched||healthView.timingLatched;
            boolean resetChord=input.resetFault&&!input.enabled&&!input.shoot&&!input.intake&&input.fieldX==0&&input.fieldY==0&&input.turn==0&&inventory.isConfirmed();
            if((fault||latch)&&resetChord&&timingOk&&snapshot!=null&&snapshot.health.drive) {
                fault=false;healthWatch.reset();
                healthView=healthWatch.update(snapshot.health,true);
            } else if(latch) {
                fault=true;reason=healthView.reason;scoring.cancel();io.stop();return;
            }
            if(!healthView.driveCommandable) {
                reason=healthView.reason;io.stop();return;
            }
            if(snapshot==null||snapshot.robot==null) {
                reason="SENSOR / TIMING FAULT";io.stop();return;
            }
            if(generation!=Long.MIN_VALUE&&snapshot.robot.getResetGeneration()<generation) {
                fault=true;reason="OLD LOCALIZATION GENERATION";scoring.cancel();io.stop();return;
            }
            if(generation!=snapshot.robot.getResetGeneration())localizationReset(now,snapshot.robot.getResetGeneration());
            if(healthView.localizationTrusted)history.add(snapshot.robot);
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
                if(tracker!=null&&tracker.size()>0) { tracker.reset(now,generation); }
            }
            PieceTrackingResult tracked=tracker==null?null:tracker.update(snapshot.pieces,history,now);
            if(inventoryRevision!=inventory.revision()) {plan=null;inventoryRevision=inventory.revision();}
            if(input.cancel)plan=null;
            if(planner!=null&&healthView.localizationTrusted&&now-lastPlan>=.1) {
                long planStarted=System.nanoTime();
                try {plan=planner.plan(snapshot.robot,inventory.load(),tracked.getPieces(),now,plan);lastPlan=now;}
                finally {planSeconds=(System.nanoTime()-planStarted)*1e-9;}
            }
            boolean targetFresh=snapshot.target!=null&&snapshot.target.fresh(now,.2,generation);
            if(targetFresh&&targetId!=snapshot.target.targetId) {hive.reset();turret.reset();targetId=snapshot.target.targetId;}
            if(targetFresh&&snapshot.target.exposureSec>hiveFrameFloor)hive.observe(snapshot.target.hive,snapshot.target.exposureSec,true);
            RobotState pivot=pivotState(snapshot.robot);
            ScoringController.State before=scoring.state();
            boolean releasing=before==ScoringController.State.FEED_ONE||before==ScoringController.State.CONFIRM_EXIT;
            if(!releasing)committedReleaseSec=Double.NaN;
            shotSolution=null;
            shotMissBoundInches=Double.NaN;
            double aimX=targetFresh?snapshot.target.x:Double.NaN,aimY=targetFresh?snapshot.target.y:Double.NaN;
            if(movingShots==null&&stationaryShots!=null&&targetFresh) {
                shotSolution=MotionShotSolver.stationary(scoring.selector(),inventory.first(),snapshot.robot,now,
                        pivotForward,pivotLeft,snapshot.turretAngle,snapshot.turretVelocity,
                        snapshot.target.x,snapshot.target.y,snapshot.target.z,maxMotion,maxTurretRate,stationaryShots);
                if(shotSolution!=null) {
                    aimX=pivot.getFieldX()+shotSolution.equivalentRange*Math.cos(shotSolution.bearing);
                    aimY=pivot.getFieldY()+shotSolution.equivalentRange*Math.sin(shotSolution.bearing);
                    shotMissBoundInches=shotSolution.errorBoundInches;
                }
            }
            if(movingShots!=null) {
                boolean trusted=motionConfidence.update(snapshot.robot,now);
                double turretDt=snapshot.acquiredSec-previousTurretSec;
                boolean turretTrusted=Double.isFinite(previousTurretAngle)&&turretDt>0&&turretDt<=movingShots.settings.motion.maxAge
                        &&Math.abs(snapshot.turretVelocity-previousTurretRate)/turretDt<=movingShots.settings.maxTurretAcceleration
                        &&Math.abs(snapshot.turretAngle-previousTurretAngle-.5*(snapshot.turretVelocity+previousTurretRate)*turretDt)
                            <=movingShots.settings.motion.headingResidual;
                previousTurretAngle=snapshot.turretAngle;previousTurretRate=snapshot.turretVelocity;previousTurretSec=snapshot.acquiredSec;
                trusted &= turretTrusted;
                double releaseSec=releasing&&Double.isFinite(committedReleaseSec)?Math.max(now,committedReleaseSec)
                        :now+movingShots.settings.releaseDelaySec;
                if(targetFresh)shotSolution=movingShots.solve(scoring.selector(),inventory.first(),snapshot.robot,now,releaseSec,
                        motionConfidence.ax,motionConfidence.ay,motionConfidence.alpha,pivotForward,pivotLeft,
                        snapshot.turretAngle,snapshot.turretVelocity,snapshot.target.x,snapshot.target.y,snapshot.target.z,trusted,releasing);
                shotConfidence=shotSolution==null?"NO CALIBRATED MOTION SOLUTION":!turretTrusted?"TURRET MOTION OUTSIDE CALIBRATION":!trusted?motionConfidence.reason:
                        !shotSolution.confident?"PREDICTED MISS EXCEEDS BOUND":"RELEASE PREDICTION WITHIN BOUNDS";
                if(shotSolution!=null) {
                    shotMissBoundInches=shotSolution.errorBoundInches;
                    double delay=shotSolution.releaseSec-now;
                    double bearingNow=shotSolution.bearing-AngleUtil.wrapRadians(shotSolution.releaseHeading-snapshot.robot.getHeadingRad())
                            -snapshot.turretVelocity*delay;
                    aimX=pivot.getFieldX()+shotSolution.equivalentRange*Math.cos(bearingNow);
                    aimY=pivot.getFieldY()+shotSolution.equivalentRange*Math.sin(bearingNow);
                }
            }
            TurretCommand aim=targetFresh&&healthView.yawCommandable?turret.calculate(TurretStateAdapter.toTurretState(pivot,
                    snapshot.turretAngle,snapshot.turretVelocity,aimX,aimY,dt,now)):null;
            if(!targetFresh||!healthView.yawCommandable)turret.reset();
            boolean scoringRelease=releasing||before==ScoringController.State.REVERSE;
            boolean scoringHealthy=(healthView.launcherCommandable||!healthView.launcherLatched&&!scoringRelease)
                    &&(healthView.inventoryTrusted||!healthView.inventoryLatched&&!scoringRelease)
                    &&(healthView.timingOk||!healthView.timingLatched&&!scoringRelease);
            ScoringController.Input shot=new ScoringController.Input();
            shot.nowSec=now;shot.acquiredSec=snapshot.acquiredSec;shot.healthy=scoringHealthy;
            shot.enabled=input.enabled;shot.start=input.shoot;shot.cancel=input.cancel||input.mechanismsInhibited;
            shot.dismissWait=input.cancelShot;
            shot.requestId=input.shotRequestId;
            shot.burst=input.burst;shot.preSpin=input.preSpin;shot.launcher=snapshot.launcher;
            shot.targetHeight=targetFresh?snapshot.target.z:Double.NaN;
            shot.impact=targetFresh?snapshot.target:null;shot.carried=inventory.load();
            shot.resetFault=input.resetFault&&!input.enabled&&!input.shoot&&!input.intake&&input.fieldX==0&&input.fieldY==0&&input.turn==0;
            shot.staged=snapshot.staged;shot.exit=snapshot.exit;shot.rpm=snapshot.rpm;
            shot.targetFresh=targetFresh;shot.targetObservedSec=targetFresh?snapshot.target.exposureSec:Double.NaN;
            shot.hiveObservedSec=shot.targetObservedSec;
            shot.targetId=targetFresh?snapshot.target.targetId:Integer.MIN_VALUE;
            shot.rangeInches=targetFresh?Math.hypot(snapshot.target.x-pivot.getFieldX(),snapshot.target.y-pivot.getFieldY()):Double.NaN;
            shot.aimReady=aim!=null&&aim.tracking&&aim.targetReachable&&healthView.yawCommandable
                    &&healthView.launcherCommandable&&healthView.localizationTrusted
                    &&Math.abs(AngleUtil.wrapRadians(aim.targetBearing-snapshot.robot.getHeadingRad()-snapshot.turretAngle))<=aimTolerance;
            if(scoringPoses!=null) {
                ShotSetupPlan setup=scoringPoses.select(snapshot.robot.getFieldX(),snapshot.robot.getFieldY(),snapshot.robot.getHeadingRad(),inventory.load());
                shot.aimReady &= setup!=null&&Math.hypot(setup.x-snapshot.robot.getFieldX(),setup.y-snapshot.robot.getFieldY())<=scoringPositionTolerance
                        &&Math.abs(AngleUtil.wrapRadians(setup.headingRad-snapshot.robot.getHeadingRad()))<=scoringHeadingTolerance;
            }
            shot.stationary=snapshot.robot.isVelocityValid()&&Math.hypot(snapshot.robot.getFieldVx(),snapshot.robot.getFieldVy())<=maxMotion
                    &&Math.abs(snapshot.robot.getAngularVelocityRadPerSec())<=maxTurretRate&&Math.abs(snapshot.turretVelocity)<=maxTurretRate;
            if(movingShots==null&&stationaryShots!=null) {
                if(!shot.stationary)stationarySince=Double.NaN;
                else if(!Double.isFinite(stationarySince))stationarySince=snapshot.acquiredSec;
                shot.stationary &= Double.isFinite(stationarySince)&&snapshot.acquiredSec-stationarySince>=stationaryShots.stableSec
                        &&shotSolution!=null&&shotSolution.confident;
                shotConfidence=shot.stationary?"CALIBRATED STATIONARY SHOT":"STOP / SETTLE / CALIBRATED STATIONARY RANGE REQUIRED";
                if(shotSolution!=null) {
                    shot.selectedShot=shotSolution.selection;shot.rangeInches=shotSolution.equivalentRange;
                    shot.releaseDelaySec=shotSolution.selection.releaseDelaySec;
                    shot.releaseSec=releasing&&Double.isFinite(committedReleaseSec)?committedReleaseSec:shotSolution.releaseSec;
                    if(releasing&&Double.isFinite(committedReleaseSec))shotSolution=shotSolution.atRelease(committedReleaseSec);
                    shot.releaseUncertaintySec=shotSolution.selection.releaseUncertaintySec;
                    double error=Math.abs(AngleUtil.wrapRadians(shotSolution.bearing-snapshot.robot.getHeadingRad()-snapshot.turretAngle));
                    shotMissBoundInches+=shot.rangeInches*Math.sin(Math.min(Math.PI/2,error));
                    shot.stationary &= shotMissBoundInches<=Math.min(stationaryShots.maxMiss,snapshot.target.lipClearance);
                    shot.aimReady &= error<=aimTolerance;
                }else shot.aimReady=false;
            }
            if(movingShots!=null) {
                // With compensation configured, even a stopped robot needs a valid calibrated solution.
                shot.stationary=false;
                shot.motionReady=shotSolution!=null&&shotSolution.confident;
                shot.aimReady &= shotSolution!=null;
                if(shotSolution!=null) {
                    shot.selectedShot=shotSolution.selection;shot.rangeInches=shotSolution.equivalentRange;
                    shot.releaseDelaySec=shotSolution.releaseSec-now;
                    shot.releaseSec=shotSolution.releaseSec;
                    shot.releaseUncertaintySec=shotSolution.selection.hasReleaseMeasurement()?shotSolution.selection.releaseUncertaintySec:movingShots.settings.releaseUncertaintySec;
                    double futureTurret=snapshot.turretAngle+snapshot.turretVelocity*shot.releaseDelaySec;
                    double physicalBearing=shotSolution.releaseHeading+futureTurret;
                    double aimError=Math.abs(AngleUtil.wrapRadians(shotSolution.bearing-physicalBearing));
                    shotMissBoundInches+=shotSolution.equivalentRange*Math.sin(Math.min(Math.PI/2,aimError));
                    double allowedMiss=snapshot.target.internalPoint?Math.min(movingShots.settings.maxMissInches,snapshot.target.lipClearance):movingShots.settings.maxMissInches;
                    if(shot.motionReady&&(!Double.isFinite(allowedMiss)||shotMissBoundInches>allowedMiss)) {
                        shot.motionReady=false;shotConfidence="AIM ERROR EXCEEDS MISS BOUND";
                    }
                    shot.aimReady &= turret.releaseAngleReachable(futureTurret)
                            &&aimError<=aimTolerance;
                }
            }
            HiveTracker.State hiveState=hive.state(now);
            shot.hiveState=hiveState;
            shot.hiveStable=hiveState==HiveTracker.State.SIDE_A_UP||hiveState==HiveTracker.State.SIDE_B_UP;
            shotTipProbability=Double.NaN;
            if(shotTipModel!=null&&shotSolution!=null&&shot.impact!=null&&Double.isFinite(scoring.cadenceAt(shotSolution.releaseSec))) {
                ShotSelector.Selection selected=shotSolution.selection;
                double uncertainty=selected.flightUncertaintySec+selected.releaseUncertaintySec;
                double cadence=scoring.cadenceAt(shotSolution.releaseSec);
                double probability=shotTipModel.getTipProbability(new ShotTipContext(inventory.load(),selected,shot.impact,
                        hiveState,cadence,shotSolution.releaseSec+selected.flightSec-uncertainty,
                        shotSolution.releaseSec+selected.flightSec+uncertainty,scoring.inFlight()));
                if(Double.isFinite(probability)&&probability>=0&&probability<=1)shotTipProbability=probability;
            }
            ScoringController.Output output=scoring.update(shot);
            if(shotSolution!=null&&!releasing&&scoring.state()==ScoringController.State.FEED_ONE&&output.feeder>0)
                committedReleaseSec=shotSolution.releaseSec;
            if(before!=ScoringController.State.WAIT_FOR_HIVE&&scoring.state()==ScoringController.State.WAIT_FOR_HIVE) {
                hive.reset();hiveFrameFloor=now;
            }
            if(input.enabled&&!input.cancel) {
                command.enabled=true;command.fieldX=input.fieldX;command.fieldY=input.fieldY;
                command.turn=assist.turn(input.turn,aim==null||!healthView.yawCommandable?0:aim.requestedChassisOmega,
                        input.unwindAssist&&healthView.yawCommandable,true);
                command.turretPower=!healthView.yawCommandable||aim==null?0:aim.turretMotorPower;
                command.acquiredSec=now;
                ShotRecipe commanded=scoring.currentRecipe();
                if(commanded!=null&&commanded.physical()&&healthView.launcherCommandable) {command.pitchRad=commanded.pitchRad;command.gapMm=commanded.gapMm;}
                command.rpm=healthView.launcherCommandable?output.rpm:0;
                command.hood=healthView.launcherCommandable?output.hood:Double.NaN;
                command.compression=healthView.launcherCommandable?output.compression:Double.NaN;
                command.transfer=healthView.launcherCommandable?output.transfer:0;
                command.feeder=healthView.launcherCommandable?output.feeder:0;
                boolean scoringAllowsIntake=scoring.state()==ScoringController.State.IDLE
                        ||scoring.state()==ScoringController.State.WAIT_FOR_HIVE
                        ||(feedStagesIsolated&&scoring.state()!=ScoringController.State.FAULT
                            &&scoring.state()!=ScoringController.State.REVERSE);
                command.intake=input.intake&&inventory.canIntake()&&healthView.inventoryTrusted&&scoringAllowsIntake?intakePower:0;
                if(input.mechanismsInhibited) {
                    command.turretPower=command.rpm=command.intake=command.transfer=command.feeder=0;
                    command.hood=command.compression=command.pitchRad=command.gapMm=Double.NaN;
                }
            }
            command.reason=reason=output.reason
                    +(movingShots!=null&&!shot.motionReady||stationaryShots!=null&&!shot.stationary?"; "+shotConfidence:"")
                    +(healthView!=null&&!"HEALTHY".equals(healthView.reason)?"; "+healthView.reason:"");
            long writeStarted=System.nanoTime();
            try {io.write(command);}finally {writeSeconds=(System.nanoTime()-writeStarted)*1e-9;}
            if(journal!=null)journal.record(now,scoring.state(),snapshot,command,scoring.currentRecipe(),shotSolution,
                    aim==null?Double.NaN:AngleUtil.wrapRadians(aim.targetBearing-snapshot.robot.getHeadingRad()),
                    motionConfidence==null?0:motionConfidence.ax,motionConfidence==null?0:motionConfidence.ay,
                    motionConfidence==null?0:motionConfidence.alpha,shotMissBoundInches);
        } catch(RuntimeException failure) {fault=true;reason="ROBOT FAULT: "+failure.getClass().getSimpleName();scoring.cancel();io.stop();}
        finally {cycleSeconds=(System.nanoTime()-cycleStarted)*1e-9;}
    }
}
