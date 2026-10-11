package org.firstinspires.ftc.teamcode.test;

import java.util.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;

/** Synthetic calibration only. Verifies math and gates, not physical shot success. */
public final class MovingShotScenarios {
    private static int checks;
    private static void check(boolean ok,String message) {checks++;if(!ok)throw new AssertionError(message);}
    private static void near(double actual,double expected,String message) {check(Math.abs(actual-expected)<.002,message+" actual="+actual);}
    private static void refuses(Runnable action,String message) {
        boolean rejected=false;try{action.run();}catch(IllegalArgumentException expected){rejected=true;}check(rejected,message);
    }
    private static ShotRecipe recipe(BallType type,double rpm,double hood) {
        return new ShotRecipe(type,rpm,hood,.6,.05,40,.05,20,140);
    }
    private static CalibratedShotTable table(boolean variable) {
        List<CalibratedShotTable.Point> points=new ArrayList<CalibratedShotTable.Point>();
        for(BallType type:BallType.values()) {
            double rpm=type==BallType.POLLEN?1000:1200;
            points.add(new CalibratedShotTable.Point(20,recipe(type,rpm,.4),.5,.01));
            points.add(new CalibratedShotTable.Point(140,recipe(type,rpm+(variable?600:0),variable?.8:.4),.5,.01));
        }
        return new CalibratedShotTable(points);
    }
    private static MotionConfidence.Limits limits() {
        return new MotionConfidence.Limits(.2,40,3,30,3,.5,.03,.1,.1,.1,.001,.01);
    }
    private static MotionShotSolver.Settings settings(double forward,double left,double miss) {
        return new MotionShotSolver.Settings(.05,.005,forward,left,.2,.3,miss,3,10,.01,limits());
    }
    private static RobotState state(double t,double x,double y,double heading,double vx,double vy,double omega,long generation) {
        return new RobotState(t,x,y,heading,vx,vy,omega,0,0,0,true,true,t,true,generation);
    }
    private static MotionShotSolver.Solution solve(MotionShotSolver solver,RobotState r,
            double ax,double ay,double alpha,double pf,double pl,double angle,double rate,boolean trusted) {
        return solver.solve(table(false),BallType.POLLEN,r,r.getTimestampSec(),r.getTimestampSec()+.05,
                ax,ay,alpha,pf,pl,angle,rate,100,0,trusted);
    }
    private static void calibration() {
        CalibratedShotTable t=table(true);
        ShotSelector.Selection middle=t.select(BallType.POLLEN,80);
        near(middle.recipe.rpm,1300,"RPM interpolation");near(middle.recipe.hood,.6,"hood interpolation");
        near(t.select(BallType.NECTAR,80).recipe.rpm,1500,"ball-specific interpolation");
        near(t.select(BallType.POLLEN,20).recipe.rpm,1000,"lower endpoint");
        near(t.select(BallType.POLLEN,140).recipe.rpm,1600,"upper endpoint");
        for(double range:new double[]{19.999,140.001,Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY})
            check(t.select(BallType.POLLEN,range)==null,"no extrapolation / invalid range "+range);
        check(t.select(null,80)==null,"unknown ball rejected");
        CalibratedShotTable.Point p=new CalibratedShotTable.Point(20,recipe(BallType.POLLEN,1000,.4),.5,.01);
        refuses(()->new CalibratedShotTable(Arrays.asList(p,p)),"duplicate knot rejected");
        refuses(()->new CalibratedShotTable(Arrays.asList(p)),"single knot not a broad range");
        refuses(()->new CalibratedShotTable.Point(20,p.recipe,.5,.5),"invalid flight interval rejected");
        refuses(()->new ScoringController(inventory(BallType.POLLEN),Arrays.asList(p.recipe,p.recipe),timing()),"fixed recipes cannot silently overwrite");
        refuses(()->new BurstPolicy(2,.1,.1,.05,.005,null),"overlap needs measured HIVE model");
    }
    private static void releasePrediction() {
        MotionShotSolver solver=new MotionShotSolver(settings(0,0,30));
        MotionShotSolver.Solution stopped=solve(solver,state(0,0,0,0,0,0,0,0),0,0,0,0,0,0,0,true);
        check(stopped!=null&&stopped.confident,"stationary calibrated solution");near(stopped.bearing,0,"stationary bearing");
        MotionShotSolver.Solution lateral=solve(solver,state(0,0,0,0,0,10,0,0),0,0,0,0,0,0,0,true);
        check(lateral!=null&&lateral.confident,"lateral solution");near(lateral.releaseY,.5,"predict release position");
        near(lateral.bearing,Math.atan2(-5.5,100),"lead against inherited lateral velocity");
        near(lateral.releaseX+lateral.releaseVx*.5+lateral.equivalentRange*Math.cos(lateral.bearing),100,"projectile landing x");
        near(lateral.releaseY+lateral.releaseVy*.5+lateral.equivalentRange*Math.sin(lateral.bearing),0,"projectile landing y");
        MotionShotSolver.Solution approaching=solve(solver,state(0,0,0,0,10,0,0,0),0,0,0,0,0,0,0,true);
        near(approaching.equivalentRange,94.5,"forward inherited velocity lowers calibrated range");
        MotionShotSolver.Solution accelerating=solve(solver,state(0,0,0,0,10,0,0,0),20,0,0,0,0,0,0,true);
        near(accelerating.releaseX,.525,"acceleration predicts release origin");near(accelerating.releaseVx,11,"acceleration predicts release velocity");
        MotionShotSolver offsetSolver=new MotionShotSolver(settings(3,0,30));
        MotionShotSolver.Solution rotating=solve(offsetSolver,state(0,0,0,0,0,0,1,0),0,0,0,5,0,0,2,true);
        near(rotating.releaseX,5*Math.cos(.05)+3*Math.cos(.15),"pivot and muzzle rotate independently");
        near(rotating.releaseY,5*Math.sin(.05)+3*Math.sin(.15),"rotating offsets y");
        near(rotating.releaseVx,-5*Math.sin(.05)-9*Math.sin(.15),"rotation-induced release velocity x");
        near(rotating.releaseVy,5*Math.cos(.05)+9*Math.cos(.15),"rotation-induced release velocity y");
        check(!solve(solver,state(0,0,0,0,0,10,0,0),0,0,0,0,0,0,0,false).confident,"untrusted motion cannot fire");
        check(!solve(new MotionShotSolver(settings(0,0,.1)),state(0,0,0,0,0,10,0,0),0,0,0,0,0,0,0,true).confident,"miss budget enforced");
        check(solver.solve(table(false),BallType.POLLEN,state(0,0,0,0,0,0,0,0),.3,.35,0,0,0,0,0,0,0,100,0,true)==null,"stale robot rejected");
        check(solve(solver,state(0,0,0,0,0,0,0,0),0,0,0,0,0,Double.NaN,0,true)==null,"invalid turret rejected");
        check(solve(solver,state(0,0,0,0,0,0,0,0),0,0,0,0,0,0,4,true)==null,"uncalibrated turret rate rejected");
        check(solve(solver,state(0,-200,0,0,0,0,0,0),0,0,0,0,0,0,0,true)==null,"uncalibrated range rejected");
        TurretController turret=new TurretController();
        check(!turret.releaseAngleReachable(10)&&!turret.releaseAngleReachable(Double.NaN),"predicted stop crossing rejected");
    }
    private static void disturbances() {
        MotionConfidence c=new MotionConfidence(limits());
        check(!c.update(state(0,0,0,0,10,0,0,0),0),"warmup first sample");
        check(!c.update(state(.05,.5,0,0,10,0,0,0),.05),"recovery needs duration");
        check(c.update(state(.11,1.1,0,0,10,0,0,0),.11),"consistent motion confident");
        check(!c.update(state(.12,1.2,0,0,25,0,0,0),.12),"push inhibits immediately");
        check(!c.update(state(.14,1.7,0,0,25,0,0,0),.14),"push recovery holds");
        check(c.update(state(.24,4.2,0,0,25,0,0,0),.24),"consistent post-push recovery");
        check(!c.update(state(.26,14.7,0,0,25,0,0,0),.26),"pose/velocity mismatch detects possible slip");
        check(!c.update(state(.26,14.7,0,0,25,0,0,0),.3),"repeated acquisition cannot earn confidence");
        check(!c.update(state(.28,15.2,0,0,25,0,0,1),.28),"reset generation restarts warmup");
        check(!c.update(state(.3,15.7,0,0,25,0,0,1),.6),"stale motion invalidates confidence");
        check(!c.update(state(.61,16,0,0,25,0,0,1),.61),"after stale input warmup required");
        check(!c.update(state(.6,16,0,0,25,0,0,1),.6),"clock rollback rejected");
        c.reset();c.update(state(0,0,0,0,0,0,0,0),0);
        check(!c.update(state(.02,0,0,.5,0,0,0,0),.02),"yaw/omega disagreement rejected");
    }
    private static PieceInventory inventory(BallType... types) {
        PieceInventory i=new PieceInventory(4);i.reconcile(Arrays.asList(types));return i;
    }
    private static ScoringController.Timing timing() {return new ScoringController.Timing(2,.15,.3,.1,.5,.2,0);}
    private static ScoringController.Input input(double t,double rpm) {
        ScoringController.Input in=new ScoringController.Input();in.nowSec=in.acquiredSec=t;
        in.healthy=in.enabled=in.start=in.staged=in.targetFresh=in.aimReady=in.hiveStable=in.stationary=true;
        in.burst=true;
        in.rpm=rpm;in.rangeInches=80;in.hiveObservedSec=in.targetObservedSec=t;in.targetId=1;
        in.hiveState=HiveTracker.State.SIDE_A_UP;return in;
    }
    private static ScoringController scorer(PieceInventory i,BurstPolicy policy) {
        ScoringController c=new ScoringController(i,table(false),timing(),.3,.2,policy);c.setFlightTiming(.02,.1);return c;
    }
    private static void firstExit(ScoringController c) {
        near(c.update(input(0,1000)).feeder,0,"initial settling");
        check(c.update(input(.06,1000)).feeder>0,"first release ready");
        ScoringController.Input in=input(.08,700);in.exit=true;
        ScoringController.Output o=c.update(in);near(o.feeder,0,"confirmed exit stops feeder despite RPM droop");
        check(c.inFlight().size()==1,"confirmed exit creates flight exactly once");
    }
    private static void bursts() {
        BurstPolicy conservative=new BurstPolicy(1,.1,.1,.02,.005,null);
        PieceInventory i=inventory(BallType.POLLEN,BallType.POLLEN,BallType.NECTAR);
        ScoringController c=scorer(i,conservative);firstExit(c);
        ScoringController.Input in=input(.09,700);in.exit=true;
        check(c.update(in).rpm>0&&c.inFlight().size()==1,"held exit not duplicated; wheel stays spinning");
        near(c.update(input(.12,700)).feeder,0,"RPM droop inhibits burst");
        near(c.update(input(.2,1000)).feeder,0,"ball in flight blocks conservative policy");
        near(c.update(input(.68,1000)).feeder,0,"impact settling interval included");
        in=input(.71,1000);in.hiveObservedSec=in.targetObservedSec=.58;
        near(c.update(in).feeder,0,"pre-impact frame cannot rearm");
        check(c.inFlight().isEmpty(),"flight retired after conservative arrival window");
        check(c.update(input(.73,1000)).feeder>0,"held trigger resumes without new start edge");
        in=input(.75,700);in.exit=true;c.update(in);check(i.first()==BallType.NECTAR,"FIFO piece types retained");
        in=input(.78,1200);near(c.update(in).rpm,1200,"next ball RPM selected while waiting");
        in=input(.8,1200);in.start=false;near(c.update(in).rpm,0,"trigger release ends burst");
        check(c.state()==ScoringController.State.IDLE&&c.inFlight().size()==1,"cancel retains flight accounting");
        in=input(.82,1200);near(c.update(in).feeder,0,"new start cannot bypass outstanding flight");

        final boolean[] safe={true};final double[] arrivals={0,0};
        BurstPolicy overlapping=new BurstPolicy(2,.1,.1,.02,.005,(next,id,hive,at,min,max,pending)->{
            arrivals[0]=min;arrivals[1]=max;
            for(BurstPolicy.Flight f:pending)if(f.targetId!=id)return false;
            return safe[0]&&hive==HiveTracker.State.SIDE_A_UP;
        });
        c=scorer(inventory(BallType.POLLEN,BallType.POLLEN,BallType.POLLEN),overlapping);firstExit(c);
        c.update(input(.12,1000));safe[0]=false;
        near(c.update(input(.2,1000)).feeder,0,"impact model veto respected");safe[0]=true;
        in=input(.22,1000);in.hiveStable=false;in.hiveState=HiveTracker.State.TIPPING;
        near(c.update(in).feeder,0,"tipping HIVE inhibits overlap");
        check(c.update(input(.24,1000)).feeder>0,"measured impact model authorizes second flight");
        near(arrivals[0],.24+.02+.5-.015,"candidate minimum arrival includes release delay");
        near(arrivals[1],.24+.02+.5+.015,"candidate maximum arrival includes uncertainty");
        in=input(.26,1000);in.exit=true;c.update(in);check(c.inFlight().size()==2,"two physical flights accounted");
        near(c.update(input(.4,1000)).feeder,0,"max in-flight bound inhibits third");
        in=input(.41,1000);in.exit=true;c.update(in);
        check(c.state()==ScoringController.State.FAULT,"unexpected extra exit during recovery faults");

        c=scorer(inventory(BallType.POLLEN,BallType.POLLEN),conservative);firstExit(c);
        in=input(.71,1000);in.hiveStable=false;c.update(in);
        in=input(1.21,1000);in.hiveStable=false;c.update(in);
        check(c.state()==ScoringController.State.FAULT,"post-impact HIVE timeout bounded");

        c=scorer(inventory(BallType.POLLEN,BallType.POLLEN),overlapping);firstExit(c);c.update(input(.12,1000));
        in=input(.2,1000);in.staged=false;c.update(in);check(c.state()==ScoringController.State.PREPARE,"next burst piece preparing");
        in=input(.21,1000);in.start=false;
        check(c.update(in).feeder==0&&c.state()==ScoringController.State.IDLE,"trigger release before feeding stops the next burst shot");

        c=scorer(inventory(BallType.POLLEN,BallType.POLLEN),conservative);
        in=input(0,1000);in.burst=false;c.update(in);
        in=input(.06,1000);in.burst=false;in.start=false;
        check(c.update(in).feeder>0,"autonomous single-start pulse still completes with burst settings configured");
        in=input(.08,1000);in.burst=false;in.start=false;in.exit=true;
        near(c.update(in).rpm,0,"single-shot request does not continue a burst");

        c=scorer(inventory(BallType.POLLEN,BallType.POLLEN),conservative);
        c.update(input(0,1000));c.update(input(.06,1000));
        near(c.update(input(.07,700)).feeder,0,"pre-exit wheel loading stops feed");
        check(c.state()==ScoringController.State.CONFIRM_EXIT,"RPM droop awaits sensed exit rather than assuming a jam");
        in=input(.08,700);in.exit=true;c.update(in);
        check(c.state()==ScoringController.State.WAIT_FOR_HIVE&&c.inFlight().size()==1,"debounced exit completes loaded shot");
        near(c.inFlight().get(0).arrivalMinSec,.55,"arrival interval includes possible sensor latency");
        near(c.inFlight().get(0).arrivalMaxSec,.59,"arrival maximum is anchored to confirmed exit");

        c=scorer(inventory(BallType.POLLEN),conservative);c.update(input(0,1000));c.update(input(.06,1000));
        in=input(.2,1000);in.exit=true;c.update(in);
        check(c.state()==ScoringController.State.FAULT&&c.inFlight().size()==1,"unexpected release delay latches calibration fault and retains flight");

        i=inventory(BallType.POLLEN);c=new ScoringController(i,table(false),
                new ScoringController.Timing(2,.15,.3,.1,.5,.2,1),.3,.2,conservative);c.setFlightTiming(.02,.1);
        c.update(input(0,1000));c.update(input(.06,1000));c.update(input(.07,700));
        check(c.update(input(.38,700)).feeder==0&&c.state()==ScoringController.State.FAULT&&!i.isConfirmed(),
                "missing exit after RPM droop cannot retry a partial shot even with retries configured");

        i=inventory(BallType.POLLEN,BallType.POLLEN);c=scorer(i,overlapping);
        c.update(input(0,1000));c.update(input(.06,1000));in=input(.07,1000);in.cancel=true;c.update(in);
        check(!i.isConfirmed()&&c.inFlight().size()==1&&!c.inFlight().get(0).confirmed,"cancelled partial release retains possible flight");
        i.reconcile(Arrays.asList(BallType.POLLEN,BallType.POLLEN));in=input(.08,1000);in.enabled=false;in.start=false;in.resetFault=true;c.update(in);
        c.update(input(.09,1000));near(c.update(input(.2,1000)).feeder,0,"reconciliation cannot bypass uncertain flight even with impact model");
    }
    private static void changingSetpoints() {
        PieceInventory i=inventory(BallType.POLLEN);ScoringController c=new ScoringController(i,table(true),timing(),.3,.2,null);
        c.setFlightTiming(.02,.1);
        ScoringController.Input in=input(0,1000);in.rangeInches=20;c.update(in);
        in=input(.04,1300);in.rangeInches=80;near(c.update(in).rpm,1300,"range updates RPM before release");
        in=input(.08,1300);in.rangeInches=80;near(c.update(in).feeder,0,"setpoint change restarts settle/dwell");
        in=input(.10,1300);in.rangeInches=80;check(c.update(in).feeder>0,"new recipe settles");
        in=input(.11,1300);in.rangeInches=100;in.selectedShot=table(true).select(BallType.POLLEN,100);
        check(c.update(in).feeder==0&&c.state()==ScoringController.State.FAULT,"release cannot silently change hood/RPM");
        c=scorer(inventory(BallType.POLLEN),null);in=input(0,1000);in.stationary=false;c.update(in);
        in=input(.06,1000);in.stationary=false;near(c.update(in).feeder,0,"uncompensated moving shot blocked");
        in=input(.08,1000);in.stationary=false;in.motionReady=true;check(c.update(in).feeder>0,"trusted compensated motion gate");

        c=new ScoringController(inventory(BallType.POLLEN),table(true),timing(),.3,.2,null);
        c.setFlightTiming(.02,.1);
        c.setRetargetTolerance(.9,.01,.01);
        for(int step=0;step<=3;step++) {
            double t=step*.02;in=input(t,1300);in.rangeInches=80+8*t;
            ScoringController.Output out=c.update(in);
            if(step==3)check(out.feeder>0,"measured retarget envelope permits continuously changing range");
        }
    }
    private static final class FakeIO implements RobotIO {
        Snapshot snapshot;Command output=new Command();
        public Snapshot read(double now,double dt){return snapshot;}
        public void write(Command c){output=c;}
        public void stop(){output=new Command();}
        void at(double t,double speed) {
            snapshot=new Snapshot(state(t,40+8*t,50,0,speed,0,0,0),t,0,0,1000,true,false,true,false,null,
                    new TargetEstimate(120,50,t,1,0,HiveTracker.State.SIDE_A_UP),Collections.emptyList());
        }
    }
    private static RobotCoordinator coordinator(FakeIO io,PieceInventory i) {
        return new RobotCoordinator(io,i,scorer(i,null),
                new PieceTracker(HomographyFloorProjection.fromCoefficients(new double[]{1,0,0,0,1,0,0,0,1}),PieceTrackerConstants.defaults()),
                new PickupPlanner(new MapTipModel().set(1,0,1),new EuclideanTravelTimeModel(),new FixedShotSetupModel(1)),
                new TurretController(),new DriveAssist(2,.3,.08),new HiveTracker(.05,.2),.02,.1,.1);
    }
    private static void integratedMotion() {
        FakeIO io=new FakeIO();PieceInventory i=inventory(BallType.POLLEN);RobotCoordinator c=coordinator(io,i);
        RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=in.shoot=true;in.fieldX=.2;
        for(int step=0;step<=6;step++){double t=step*.02;io.at(t,8);c.cycle(in,t,.02);near(io.output.feeder,0,"stationary gate remains without solver "+step);}
        io=new FakeIO();i=inventory(BallType.POLLEN);c=coordinator(io,i);c.setMovingShots(settings(0,0,30));
        double firstFeed=Double.NaN;
        for(int step=0;step<=5;step++) {
            double t=step*.02;io.at(t,8);c.cycle(in,t,.02);
            if(io.output.feeder>0)firstFeed=t;
        }
        check(Double.isFinite(firstFeed)&&c.shotSolution.confident,"integrated moving release above old speed guard");
        near(io.output.fieldX,.2,"moving fire retains driver translation");
        io.at(.12,25);c.cycle(in,.12,.02);
        check(io.output.feeder==0&&c.scoring.state()==ScoringController.State.FAULT&&!i.isConfirmed(),"push during release stops and requires reconciliation");

        io=new FakeIO();i=inventory(BallType.POLLEN);c=coordinator(io,i);
        MotionShotSolver.Solution base=new MotionShotSolver(settings(0,0,30)).solve(table(false),BallType.POLLEN,
                state(0,40,50,0,8,0,0,0),0,.05,0,0,0,0,0,.01,0,120,50,true);
        c.setMovingShots(settings(0,0,base.errorBoundInches+.1));
        for(int step=0;step<=5;step++) {
            double t=step*.02;io.at(t,8);
            RobotIO.Snapshot original=io.snapshot;
            io.snapshot=new RobotIO.Snapshot(original.robot,t,.01,0,1000,true,false,true,false,null,original.target,Collections.emptyList());
            c.cycle(in,t,.02);
        }
        check(io.output.feeder==0&&c.shotConfidence.equals("AIM ERROR EXCEEDS MISS BOUND"),
                "measured alignment error cannot consume the miss budget twice");
    }
    public static void main(String[] args) {
        calibration();releasePrediction();disturbances();bursts();changingSetpoints();integratedMotion();
        System.out.println("MOVING SHOT CHECKS PASSED ("+checks+")");
    }
}
