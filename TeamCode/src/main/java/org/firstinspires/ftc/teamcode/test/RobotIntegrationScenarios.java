package org.firstinspires.ftc.teamcode.test;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;

/** Deterministic integrated behavior and fault injection, with simulation measurements only. */
public final class RobotIntegrationScenarios {
    private static int checks;
    private static void check(boolean yes,String name){checks++;if(!yes)throw new AssertionError(name);}
    private static void near(double a,double b,String name){check(Math.abs(a-b)<1e-8,name+" actual="+a);}
    private static RobotState pose(double t,double x,double y,double h,long generation) {
        return new RobotState(t,x,y,h,0,0,0,0,0,0,true,true,t,true,generation);
    }
    private static PieceInventory inventory() {
        PieceInventory i=new PieceInventory(4);i.reconcile(Arrays.asList(BallType.POLLEN,BallType.NECTAR));return i;
    }
    private static ScoringController scoring(PieceInventory i,int retries) {
        return new ScoringController(i,Arrays.asList(new ShotRecipe(BallType.POLLEN,1000,.4,.6,.1,50,.1),
                new ShotRecipe(BallType.NECTAR,1200,.5,.7,.1,50,.1)),
                new ScoringController.Timing(.5,.1,.2,.1,.5,.2,retries),.3,.2);
    }
    private static ScoringController.Input shot(double t) {
        ScoringController.Input in=new ScoringController.Input();in.nowSec=in.acquiredSec=t;
        in.healthy=in.enabled=in.staged=in.aimReady=in.targetFresh=in.stationary=in.hiveStable=true;
        in.rpm=1000;in.hiveObservedSec=in.targetObservedSec=t;return in;
    }
    private static void stopped(ScoringController.Output o,String message){near(o.rpm,0,message+" rpm");near(o.feeder,0,message+" feeder");near(o.transfer,0,message+" transfer");}
    private static void scoringCycle() {
        PieceInventory i=inventory();ScoringController c=scoring(i,0);
        ScoringController.Input in=shot(0);in.start=true;
        near(c.update(in).feeder,0,"initial settle");check(c.state()==ScoringController.State.PREPARE,"prepare");
        in=shot(.05);in.start=true;near(c.update(in).feeder,0,"RPM persistence");
        in=shot(.11);in.start=true;near(c.update(in).feeder,.3,"one measured feed");
        check(c.state()==ScoringController.State.FEED_ONE,"feed state");check(i.pieces().size()==2,"no timer decrement");
        in=shot(.15);in.exit=true;in.start=true;stopped(c.update(in),"sensed exit stops feed");
        check(i.first()==BallType.NECTAR&&i.pieces().size()==1,"ordered exit confirmation");
        check(c.state()==ScoringController.State.WAIT_FOR_HIVE,"wait after shot");
        in=shot(.3);in.start=true;in.hiveObservedSec=in.targetObservedSec=.1;c.update(in);
        check(c.state()==ScoringController.State.WAIT_FOR_HIVE,"pre-release readiness rejected");
        in=shot(.32);in.start=true;c.update(in);check(c.state()==ScoringController.State.IDLE,"new frame reacquired");
        in=shot(.35);in.start=true;stopped(c.update(in),"held trigger no next shot");
        in=shot(.4);c.update(in);in=shot(.41);in.start=true;in.rpm=1200;
        near(c.update(in).rpm,1200,"next piece recipe");
        in=shot(.42);in.cancel=true;stopped(c.update(in),"prepare cancel");check(c.state()==ScoringController.State.IDLE,"cancel before release safe");
    }
    private static void scoringFaults() {
        for(int gate=0;gate<5;gate++) {
            PieceInventory i=inventory();ScoringController c=scoring(i,0);ScoringController.Input in=shot(0);in.start=true;c.update(in);
            in=shot(.11);
            if(gate==0)in.aimReady=false;if(gate==1)in.targetFresh=false;if(gate==2)in.stationary=false;
            if(gate==3)in.hiveStable=false;if(gate==4)in.rpm=700;
            near(c.update(in).feeder,0,"each readiness gate inhibits "+gate);
            check(c.state()==ScoringController.State.PREPARE,"gate stays prepare "+gate);
        }
        PieceInventory i=inventory();ScoringController c=scoring(i,0);ScoringController.Input in=shot(0);in.start=true;c.update(in);
        in=shot(.11);c.update(in);in=shot(.12);in.targetFresh=false;stopped(c.update(in),"release readiness loss");
        check(c.state()==ScoringController.State.FAULT&&!i.isConfirmed(),"partial release uncertain");
        c.cancel();check(c.state()==ScoringController.State.FAULT,"public cancel preserves latch");
        in=shot(.15);in.resetFault=true;in.enabled=false;c.update(in);check(c.state()==ScoringController.State.FAULT,"no reset of uncertain queue");
        i.reconcile(Arrays.asList(BallType.POLLEN,BallType.NECTAR));in=shot(.16);in.enabled=false;c.update(in);
        in=shot(.17);in.resetFault=true;c.update(in);check(c.state()==ScoringController.State.FAULT,"enabled reset refused");
        in=shot(.18);in.enabled=false;c.update(in);in=shot(.19);in.enabled=false;in.resetFault=true;c.update(in);
        check(c.state()==ScoringController.State.IDLE,"disabled deliberate reset after reconciliation");
        c=scoring(inventory(),0);in=shot(0);c.update(in);in=shot(.1);in.acquiredSec=-1;stopped(c.update(in),"stale sensor");check(c.state()==ScoringController.State.FAULT,"stale latched");
        c=scoring(inventory(),0);in=shot(1);c.update(in);in=shot(.9);stopped(c.update(in),"clock rollback");
        c=scoring(inventory(),0);in=shot(0);in.exit=true;in.start=true;c.update(in);check(c.state()==ScoringController.State.FAULT,"startup blocked exit");
        c=scoring(inventory(),0);in=shot(0);c.update(in);in=shot(.1);in.exit=true;c.update(in);check(c.state()==ScoringController.State.FAULT,"unexpected exit faults");
        c=scoring(inventory(),1);in=shot(0);in.start=true;in.aimReady=false;c.update(in);
        in=shot(.51);in.aimReady=false;stopped(c.update(in),"stop before reversal");check(c.state()==ScoringController.State.REVERSE,"bounded recovery");
        in=shot(.55);near(c.update(in).feeder,-.2,"measured reverse power");
        in=shot(.62);in.aimReady=false;c.update(in);check(c.state()==ScoringController.State.PREPARE,"retry prepare");
        in=shot(1.13);in.aimReady=false;c.update(in);check(c.state()==ScoringController.State.FAULT,"retry exhausted");
        c=scoring(inventory(),0);in=shot(0);in.start=true;c.update(in);c.update(shot(.11));in=shot(.12);in.cancel=true;
        stopped(c.update(in),"release cancellation");check(c.state()==ScoringController.State.FAULT,"cancel during release requires inspection");
    }
    private static SweptCaptureFeasibility footprint(){return new SweptCaptureFeasibility(144,144,5,null);}
    private static void inventoryAndHive() {
        PieceInventory i=new PieceInventory(2);long a=i.reserve(BallType.POLLEN),b=i.reserve(BallType.NECTAR);
        check(a>0&&b>a&&!i.canIntake(),"reservations occupy capacity");check(i.reserve(BallType.POLLEN)<0,"capacity rejected");
        check(i.confirmEntry(a)&&!i.confirmEntry(a),"entry exactly once");check(!i.confirmExit(BallType.NECTAR),"wrong exit keeps queue");
        i.cancelReservation(b);check(i.canIntake(),"cancel reservation frees slot");check(i.confirmExit(BallType.POLLEN),"correct exit");
        i.markUncertain();check(!i.canIntake()&&i.first()==null,"uncertainty inhibits");i.reconcile(Collections.emptyList());check(i.isConfirmed(),"measured reconciliation");
        HiveTracker h=new HiveTracker(.1,.2);h.observe(HiveTracker.State.SIDE_A_UP,0,true);
        h.observe(HiveTracker.State.SIDE_A_UP,0,true);check(h.state(.1)==HiveTracker.State.UNKNOWN,"duplicates not stable");
        h.observe(HiveTracker.State.SIDE_A_UP,.11,true);check(h.state(.11)==HiveTracker.State.SIDE_A_UP,"distinct settled observations");
        h.observe(HiveTracker.State.TIPPING,.12,true);check(h.state(.12)==HiveTracker.State.TIPPING,"tip immediately inhibits");
        h.observe(HiveTracker.State.SIDE_B_UP,.13,true);check(h.state(.13)==HiveTracker.State.UNKNOWN,"new side must settle");
        h.observe(HiveTracker.State.SIDE_B_UP,.24,true);check(h.state(.24)==HiveTracker.State.SIDE_B_UP,"new side settled");
        check(h.state(.5)==HiveTracker.State.UNKNOWN,"old hive unavailable");
    }
    private static FixedRouteAuto route(FixedRouteAuto.Waypoint... w) {
        return new FixedRouteAuto(w,new FixedRouteAuto.Waypoint(20,20,0,5,false),footprint(),30,25,1,.05,.05,1,.3);
    }
    private static void autonomous() {
        FixedRouteAuto a=route(new FixedRouteAuto.Waypoint(40,20,0,3,false,true));
        FixedRouteAuto.Command c=a.update(pose(0,30,20,0,0),0,false,false,false);check(c.xPower>0&&a.intakeRequested(),"pickup leg starts");
        a.update(pose(.1,35,20,0,0),.1,false,true,false);
        c=a.update(pose(.2,40,20,0,0),.2,false,false,false);check(c.reason.equals("NEXT LEG"),"early pickup event retained");
        c=a.update(pose(.3,40,20,0,0),.3,false,false,false);check(c.action==FixedRouteAuto.Action.PARK&&c.xPower<0,"park motion continues");
        c=a.update(pose(.4,20,20,0,0),.4,false,false,false);check(c.action==FixedRouteAuto.Action.DONE,"park completes");
        a=route(new FixedRouteAuto.Waypoint(40,20,0,1,true));c=a.update(pose(0,40,20,0,0),0,false,false);check(c.action==FixedRouteAuto.Action.SHOOT,"shot requests");
        c=a.update(pose(.5,40,20,0,0),.5,false,false);check(c.action==FixedRouteAuto.Action.SHOOT,"wait for physical release");
        c=a.update(pose(1.1,40,20,0,0),1.1,false,false);check(c.action==FixedRouteAuto.Action.PARK,"shot timeout parks");
        a=route(new FixedRouteAuto.Waypoint(40,20,0,29,false));a.update(pose(0,30,20,0,0),0,false,false);
        c=a.update(pose(25,30,20,0,0),25,false,false);check(c.action==FixedRouteAuto.Action.PARK,"park margin");
        c=a.update(pose(30,30,20,0,0),30,false,false);check(c.action==FixedRouteAuto.Action.DONE&&c.xPower==0,"hard deadline stops");
        a=route();check(a.update(pose(0,30,20,0,0),0,false,false).action==FixedRouteAuto.Action.PARK,"empty route parks");
        a=route();a.update(pose(1,30,20,0,0),1,false,false);check(a.update(pose(.9,30,20,0,0),.9,false,false).action==FixedRouteAuto.Action.FAULT,"auto clock fault");
        a=route();a.update(pose(0,30,20,0,0),0,false,false);check(a.update(pose(.1,30,20,0,1),.1,false,false).action==FixedRouteAuto.Action.FAULT,"auto generation fault");
        a=route();check(a.update(pose(0,30,20,0,0),1,false,false).action==FixedRouteAuto.Action.FAULT,"auto stale fault");
    }
    private static void projectionAndPickup() {
        RobotStateHistory h=new RobotStateHistory();h.add(pose(0,20,30,Math.PI/2,0));h.add(pose(.1,20,30,Math.PI/2,0));
        TurretPoseHistory t=new TurretPoseHistory();t.add(0,Math.PI/2,0);t.add(.1,Math.PI/2,0);
        CameraTargetTransform transform=new CameraTargetTransform(new double[]{1,0,0,0,1,0,0,0,1},2,0,1,0,1,100);
        TargetEstimate p=transform.project(10,0,0,.05,1,HiveTracker.State.SIDE_A_UP,h,t,0);
        check(p!=null,"history projection");near(p.x,9,"exposure yaw and camera offset x");near(p.y,32,"pivot offset y");
        check(transform.project(10,0,0,.05,1,HiveTracker.State.SIDE_A_UP,h,t,1)==null,"camera reset mismatch");
        check(transform.project(-10,0,0,.05,1,HiveTracker.State.SIDE_A_UP,h,t,0)==null,"behind camera rejected");
        check(transform.project(10,0,0,.5,1,HiveTracker.State.SIDE_A_UP,h,t,0)==null,"missing exposure rejected");
        SweptCaptureFeasibility blocked=new SweptCaptureFeasibility(144,144,5,new double[][]{{40,40,50,50}});
        check(!blocked.segmentSafe(20,45,70,45),"body swept obstacle");check(!blocked.segmentSafe(2,20,20,20),"wall clearance");check(blocked.segmentSafe(20,20,70,20),"clear corridor");
        PreferredPoseShotSetup shots=new PreferredPoseShotSetup(new PreferredPoseShotSetup.ShotPose[]{new PreferredPoseShotSetup.ShotPose(30,20,0)},20,1,load->load.total()*2,footprint());
        near(shots.estimateSeconds(30,20,0,new BallLoad(2,0)),4,"full load cycle priced");
        for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,-1,0}) {
            check(Double.isNaN(new PreferredPoseShotSetup(new PreferredPoseShotSetup.ShotPose[]{new PreferredPoseShotSetup.ShotPose(30,20,0)},invalid,1,1).estimateSeconds(20,20,0,new BallLoad(1,0))),"invalid measured speed rejected");
        }
        TrackedPiece piece=new TrackedPiece(1,BallType.POLLEN,PieceOwnership.NEUTRAL,40,20,1,0,true);
        PickupTarget target=new PickupTarget(piece,45,20,0);
        PickupPlan plan=PickupPlan.of(PickupPlan.Decision.PICKUP,Arrays.asList(target),new BallLoad(1,0),1,1,1,1,45,20,0);
        PickupAssist a=new PickupAssist(new PickupAssist.Settings(1,.05,.05,1,.3,2,footprint()),plan,pose(0,20,20,0,0),0,0);
        check(!a.update(pose(0,20,20,0,0),0,0).intake,"stage before capture");
        a.update(pose(.1,35,20,0,0),.1,0);check(a.update(pose(.2,40,20,0,0),.2,0).intake,"capture intake");
        a.update(pose(.3,42,20,0,0),.3,1);check(a.update(pose(.4,45,20,0,0),.4,1).done,"confirmed pickup retained");
        a=new PickupAssist(new PickupAssist.Settings(1,.05,.05,1,.3,2,footprint()),plan,pose(0,20,20,0,0),0,0);
        check(a.update(pose(.1,20,20,0,1),.1,0).fault,"pickup reset stops");
        PickupPlanner planner=new PickupPlanner(new MapTipModel().set(1,0,1),new EuclideanTravelTimeModel(20,1,.2),shots,footprint());
        plan=planner.plan(pose(0,30,20,0,0),new BallLoad(1,0),Collections.emptyList(),0,null);
        check(plan.getShotSetup()!=null&&plan.withUtility(.1).getShotSetup()!=null,"shot pose contract retained");
    }
    private static final class FakeIO implements RobotIO {
        Snapshot next;Command output=new Command();int stops,reads,writes;boolean throwRead;
        public Snapshot read(double now,double dt){reads++;if(throwRead)throw new IllegalStateException("disconnect");return next;}
        public void write(Command c){writes++;output=c;}
        public void stop(){stops++;output=new Command();}
        void at(double now,long generation,boolean entry,double acquired,double turretAngle,double bearing) {
            next=new Snapshot(pose(now,60,50,0,generation),acquired,turretAngle,0,1000,true,entry,true,false,
                    BallType.POLLEN,new TargetEstimate(60+40*Math.cos(bearing),50+40*Math.sin(bearing),now,1,generation,HiveTracker.State.SIDE_A_UP),Collections.emptyList());
        }
    }
    private static void coordinator() {
        FakeIO io=new FakeIO();PieceInventory i=inventory();ScoringController s=scoring(i,0);
        TurretControlProfile profile=new TurretControlProfile(-2.9,2.9,-2.8,2.8,2.5,2,1,1,1,1,.1,.1,.3,1,2);
        RobotCoordinator c=new RobotCoordinator(io,i,s,new PieceTracker(HomographyFloorProjection.fromCoefficients(new double[]{1,0,0,0,1,0,0,0,1}),PieceTrackerConstants.defaults()),
                new PickupPlanner(new MapTipModel().set(2,0,1),new EuclideanTravelTimeModel(),new FixedShotSetupModel(1)),
                new TurretController(profile),new DriveAssist(2,.3,.08),new HiveTracker(.1,.2),.02,.1,.1);
        RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=true;in.shoot=true;
        for(int step=0;step<12;step++) {double now=step*.02;io.at(now,0,false,now,0,1);c.cycle(in,now,.02);near(io.output.feeder,0,"profile reference is not physical aim "+step);}
        check(s.state()==ScoringController.State.PREPARE,"geometric aim gate");
        in.cancelShot=true;in.fieldX=.2;io.at(.24,0,false,.24,0,0);c.cycle(in,.24,.02);near(io.output.fieldX,.2,"shot cancellation retains park drive");
        in.enabled=false;io.at(.26,0,false,.26,0,0);c.cycle(in,.26,.02);check(!io.output.enabled&&io.output.fieldX==0,"enable release zeroes outputs");
        in.cancelShot=false;in.shoot=false;in.enabled=true;in.intake=true;
        io.at(.28,0,false,.28,0,0);c.cycle(in,.28,.02);io.at(.3,0,true,.3,0,0);c.cycle(in,.3,.02);
        check(i.pieces().size()==3,"sensed intake once");io.at(.32,0,true,.32,0,0);c.cycle(in,.32,.02);check(i.pieces().size()==3,"held entry not duplicate");
        io.at(.34,1,false,.34,0,0);c.cycle(in,.34,.02);check(!c.history.sampleAt(.3).isPresent(),"coherent generation clears history");
        io.at(.36,1,false,0,0,0);c.cycle(in,.36,.02);check(io.output.rpm==0&&io.output.intake==0,"stale snapshot stop");
        io.at(.38,1,false,.38,0,0);c.cycle(in,.38,.02);check(!io.output.enabled,"sensor fault latches");
        in.enabled=false;in.intake=false;in.resetFault=true;in.fieldX=0;io.at(.4,1,false,.4,0,0);c.cycle(in,.4,.02);
        in.resetFault=false;in.enabled=true;io.at(.42,1,false,.42,0,0);c.cycle(in,.42,.02);check(io.output.enabled,"deliberate reset resumes");
        int previousReads=io.reads,previousWrites=io.writes;
        io.throwRead=true;c.cycle(in,.44,.02);check(!io.output.enabled,"SDK exception zero");
        check(io.reads==previousReads+1&&io.writes==previousWrites,"failed snapshot never publishes actuator request");
        DriveAssist assist=new DriveAssist(2,.3,.08);near(assist.turn(.4,1,true,true),.4,"driver rotation wins");near(assist.turn(0,1,true,true),.3,"assist power cap");near(assist.turn(0,1,false,true),0,"assist deliberate hold");
    }
    private static void profilesAndReleaseTimeouts() {
        TurretControlProfile profile=new TurretControlProfile(-2.9,2.9,-2.8,2.8,2.5,2,1,1,1,1,.1,.1,.3,1,2);
        TurretController c=new TurretController(profile);double angle=0,velocity=0;
        for(int step=0;step<180;step++) {
            double now=step*.02;
            TurretCommand command=c.calculate(TurretStateAdapter.toTurretState(pose(now,60,50,0,0),angle,velocity,
                    60+40*Math.cos(1),50+40*Math.sin(1),.02,now));
            check(Math.abs(command.desiredTurretVelocity-velocity)<=profile.maxAcceleration*.02+1e-8,"reference acceleration bound "+step);
            check(Math.abs(command.desiredTurretVelocity)<=profile.maxSpeed+1e-8,"reference speed bound "+step);
            check(Math.abs(command.turretMotorPower)<=profile.MAX_MOTOR_POWER,"measured power cap "+step);
            angle=command.desiredTurretAngle;velocity=command.desiredTurretVelocity;
        }
        check(Math.abs(angle-1)<.001,"profile reaches bearing");
        TurretCommand stopped=c.calculate(TurretStateAdapter.toTurretState(pose(4,60,50,0,0),angle,velocity,100,50,1,4));
        check(!stopped.tracking&&stopped.turretMotorPower==0,"invalid loop clears trajectory");
        TurretCommand reset=c.calculate(TurretStateAdapter.toTurretState(pose(4.02,60,50,0,1),0,0,100,90,.02,4.02));
        check(Math.abs(reset.desiredTurretVelocity)<=profile.maxAcceleration*.02,"reset starts bounded trajectory");
        PieceInventory i=inventory();ScoringController s=scoring(i,0);ScoringController.Input in=shot(0);in.start=true;s.update(in);
        s.update(shot(.11));s.update(shot(.22));check(s.state()==ScoringController.State.CONFIRM_EXIT,"feed timer stops and waits for exit");
        check(i.pieces().size()==2,"no inferred timed exit");s.update(shot(.43));check(s.state()==ScoringController.State.FAULT,"no exit times out");
        s=scoring(inventory(),0);in=shot(0);in.start=true;s.update(in);s.update(shot(.11));in=shot(.12);in.exit=true;s.update(in);
        in=shot(.63);in.hiveStable=false;s.update(in);check(s.state()==ScoringController.State.FAULT,"hive reacquisition bounded");
        s=new ScoringController(inventory(),Arrays.asList(new ShotRecipe(BallType.POLLEN,1000,.4,.6,.1,50,.1,10,20)),
                new ScoringController.Timing(.5,.1,.2,.1,.5,.2,0));
        in=shot(0);in.start=true;in.rangeInches=30;s.update(in);in=shot(.11);in.rangeInches=30;near(s.update(in).feeder,0,"uncalibrated target range inhibits");
        in=shot(.12);in.rangeInches=15;in.targetObservedSec=.2;near(s.update(in).feeder,0,"future frame inhibits");
        in=shot(.13);in.rangeInches=15;check(s.update(in).feeder>0,"measured range and readiness authorize");
    }
    private static void workerLifecycle() throws Exception {
        CountDownLatch reading=new CountDownLatch(1),release=new CountDownLatch(1);
        AtomicReference<Thread> owner=new AtomicReference<Thread>();
        HuskyLensWorker worker=new HuskyLensWorker(time->{
            owner.set(Thread.currentThread());reading.countDown();boolean done=false;
            while(!done)try{release.await();done=true;}catch(InterruptedException ignored){/* Model a non-cancellable SDK read. */}
            return Arrays.asList(PieceObservation.of(1,20,10,4,4,time));
        },System.nanoTime());
        worker.start();boolean started=reading.await(1,TimeUnit.SECONDS);worker.close();release.countDown();
        check(started,"camera read dispatched");owner.get().join(1000);
        check(!owner.get().isAlive()&&worker.latest().isEmpty(),"closed worker cannot publish delayed read");
        BooleanDebouncer sensor=new BooleanDebouncer(.05);
        check(!sensor.update(false,0),"debounce initial clear");check(!sensor.update(true,.01),"short rising edge held");
        check(!sensor.update(false,.03),"bounce resets dwell");check(!sensor.update(true,.04),"new edge starts dwell");
        check(sensor.update(true,.1),"settled edge accepted");check(sensor.update(false,.11),"falling edge also debounced");
        check(!sensor.update(false,.17),"settled falling edge accepted");
        boolean refused=false;try{sensor.update(false,.1);}catch(IllegalStateException expected){refused=true;}
        check(refused,"debounce clock fault");
    }
    public static void main(String[] args) throws Exception {scoringCycle();scoringFaults();inventoryAndHive();autonomous();projectionAndPickup();coordinator();profilesAndReleaseTimeouts();workerLifecycle();System.out.println("ROBOT INTEGRATION CHECKS PASSED ("+checks+")");}
}
