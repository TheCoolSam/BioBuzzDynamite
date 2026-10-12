package org.firstinspires.ftc.teamcode.test;

import java.util.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.match.AllianceColor;

/** Synthetic fault-injection contracts. Does not establish physical calibration. */
public final class CompetitionReadinessScenarios {
    private static int checks;
    private static void check(boolean pass,String message) {checks++;if(!pass)throw new AssertionError(message);}
    private static void near(double value,double expected,String message) {check(Math.abs(value-expected)<1e-8,message+" actual="+value);}
    private static void rejects(Runnable action,String message) {
        boolean rejected=false;try {action.run();}catch(IllegalArgumentException expected){rejected=true;}check(rejected,message);
    }
    private static RobotState pose(double t,double x,double y,double heading,long generation,double speed) {
        return new RobotState(t,x,y,heading,speed,0,0,0,0,0,true,true,t,true,generation);
    }
    private static PieceInventory inventory() {
        PieceInventory p=new PieceInventory(4);p.reconcile(Arrays.asList(BallType.POLLEN,BallType.POLLEN));return p;
    }
    private static ScoringController scorer(PieceInventory p) {
        ShotRecipe recipe=new ShotRecipe(BallType.POLLEN,1000,.4,.6,.05,40,.05);
        return new ScoringController(p,(type,range)->Double.isFinite(range)?new ShotSelector.Selection(recipe,Double.NaN,Double.NaN):null,
                new ScoringController.Timing(1,.15,.3,.1,1,.2,0),.3,.2,null);
    }
    private static ScoringController.Input shot(double t,long id) {
        ScoringController.Input in=new ScoringController.Input();in.nowSec=in.acquiredSec=in.hiveObservedSec=in.targetObservedSec=t;
        in.healthy=in.enabled=in.staged=in.aimReady=in.targetFresh=in.stationary=in.hiveStable=true;
        in.rpm=1000;in.rangeInches=80;in.requestId=id;return in;
    }
    private static void actions() {
        ScoringController c=scorer(inventory());ScoringController.Input in=shot(0,1);in.rangeInches=Double.NaN;c.update(in);
        check(c.state()==ScoringController.State.IDLE&&c.acceptedRequestId()==0,"missing target preserves pending action");
        c.update(shot(.02,1));check(c.acceptedRequestId()==1&&c.state()==ScoringController.State.PREPARE,"late target accepts same action");
        c.update(shot(.08,1));check(c.state()==ScoringController.State.FEED_ONE,"accepted action feeds after readiness");
        in=shot(.10,1);in.exit=true;c.update(in);check(c.completedRequestId()==1,"exit completes matching action once");
        c.update(shot(.12,2));check(c.acceptedRequestId()==1,"busy scorer keeps next action pending");
        c.update(shot(.18,2));check(c.state()==ScoringController.State.IDLE,"reacquisition completes without consuming next action");
        c.update(shot(.20,2));check(c.acceptedRequestId()==2&&c.state()==ScoringController.State.PREPARE,"next action accepted after idle");
        c.update(shot(.26,2));in=shot(.28,2);in.exit=true;c.update(in);c.update(shot(.36,2));
        in=shot(.38,2);in.start=true;c.update(in);check(c.state()==ScoringController.State.IDLE,"duplicate completed ID never replays even with start edge");
        c=scorer(inventory());c.update(shot(0,4));c.update(shot(.06,4));in=shot(.08,4);in.exit=true;c.update(in);
        check(c.state()==ScoringController.State.WAIT_FOR_HIVE,"confirmed release waits");
        in=shot(.10,0);in.dismissWait=true;ScoringController.Output dismissed=c.update(in);
        check(c.state()==ScoringController.State.IDLE&&c.completedRequestId()==4&&c.failedRequestId()==0,"dismissWait finishes hive wait without faulting");
        near(dismissed.feeder,0,"dismissed wait never feeds");
        c=scorer(inventory());c.setHoldRpm(400);
        in=shot(0,0);ScoringController.Output held=c.update(in);
        near(held.rpm,400,"configured hold spins while a piece is staged");near(held.feeder,0,"hold never feeds");
        in=shot(.02,0);in.enabled=false;near(c.update(in).rpm,0,"disable stops the flywheel");
        c=scorer(inventory());c.setHoldRpm(400);c.update(shot(0,5));c.update(shot(.06,5));in=shot(.08,5);in.exit=true;
        near(c.update(in).rpm,400,"hold continues through WAIT_FOR_HIVE with remaining inventory");
        PieceInventory empty=new PieceInventory(4);empty.reconcile(Collections.<BallType>emptyList());
        ScoringController last=scorer(empty);last.setHoldRpm(400);in=shot(0,0);in.staged=false;
        near(last.update(in).rpm,0,"empty unstaged inventory stops the flywheel");
        c=scorer(inventory());c.update(shot(0,7));in=shot(.01,7);in.cancel=true;c.update(in);
        check(c.failedRequestId()==7&&c.completedRequestId()==0,"cancel before release reports failed action");
        c.update(shot(.02,7));check(c.state()==ScoringController.State.IDLE,"cancelled accepted ID never retries");
        c=scorer(inventory());c.update(shot(0,3));c.update(shot(.06,3));in=shot(.07,3);in.cancel=true;c.update(in);
        check(c.state()==ScoringController.State.FAULT&&c.failedRequestId()==3,"ambiguous interrupted release faults action");
    }
    private static FixedRouteAuto auto(SweptCaptureFeasibility bounds,FixedRouteAuto.Departure departure) {
        return new FixedRouteAuto(new FixedRouteAuto.Waypoint[]{new FixedRouteAuto.Waypoint(24,40,0,3,true),
                new FixedRouteAuto.Waypoint(24,40,0,3,true)},new FixedRouteAuto.Waypoint(30,40,0,3,false),bounds,
                30,25,.5,.03,.1,1,.3,departure);
    }
    private static void departures() {
        SweptCaptureFeasibility bounds=new SweptCaptureFeasibility(144,144,Math.hypot(9,9),null);
        FixedRouteAuto.Departure d=new FixedRouteAuto.Departure(24,40,0,9,9,.03,2);
        FixedRouteAuto a=auto(bounds,d);
        FixedRouteAuto.Command cmd=a.update(pose(0,9,40,0,0,0),0,0L,false,false);
        check(cmd.action==FixedRouteAuto.Action.DRIVE&&cmd.xPower>0,"legal wall-contact start departs inward");
        near(cmd.turnPower,0,"departure forbids commanded rotation");check(a.departing()&&!a.intakeRequested()&&a.shotRequestId()==0,"departure has no mechanism actions");
        a.update(pose(.1,24,40,0,0,0),.1,0L,false,false);check(!a.departing(),"full-clearance arrival ends departure");
        a.update(pose(.2,24,40,0,0,0),.2,0L,false,false);check(a.shotRequestId()==1,"first shot action ID");
        cmd=a.update(pose(.3,24,40,0,0,0),.3,99L,false,false);check(cmd.action==FixedRouteAuto.Action.SHOOT,"unrelated completion cannot advance route");
        a.update(pose(.4,24,40,0,0,0),.4,1L,false,false);
        a.update(pose(.5,24,40,0,0,0),.5,1L,false,false);check(a.shotRequestId()==2,"next waypoint has distinct ID");
        a=auto(bounds,d);cmd=a.update(pose(0,9,40,.08,0,0),0,0L,false,false);check(cmd.action==FixedRouteAuto.Action.FAULT,"heading deviation stops departure");
        a=auto(bounds,d);a.update(pose(0,9,40,0,0,0),0,0L,false,false);
        cmd=a.update(pose(2.1,10,40,0,0,0),2.1,0L,false,false);check(cmd.action==FixedRouteAuto.Action.FAULT,"departure has bounded timeout");
        a=auto(bounds,d);a.update(pose(0,9,40,0,0,0),0,0L,false,false);
        cmd=a.update(pose(.1,10,40,0,1,0),.1,0L,false,false);check(cmd.action==FixedRouteAuto.Action.FAULT,"departure localization reset faults");
        check(!bounds.departureSafe(8,40,24,40,0,9,9),"actual out-of-field rectangle refused");
        check(!bounds.departureSafe(9,40,24,40,0,10,10),"understated enclosing radius refused");
        auto(bounds,new FixedRouteAuto.Departure(9,40,0,9,9,.03,2));
        check(bounds.poseSafe(9,40,0,9,9),"wall-adjacent oriented endpoint accepted");
        rejects(()->auto(bounds,new FixedRouteAuto.Departure(8,40,0,9,9,.03,2)),"oriented endpoint still refuses out-of-field rectangle");
        SweptCaptureFeasibility blocked=new SweptCaptureFeasibility(144,144,Math.hypot(9,9),new double[][]{{24,39,25,41}});
        check(!blocked.departureSafe(9,40,40,40,0,9,9),"departure still protects obstacles");
        SweptCaptureFeasibility oriented=new SweptCaptureFeasibility(144,144,Math.hypot(9,9),null,9,9);
        FixedRouteAuto wallPark=new FixedRouteAuto(new FixedRouteAuto.Waypoint[]{new FixedRouteAuto.Waypoint(24,40,0,3,false)},
                new FixedRouteAuto.Waypoint(9,40,0,3,false),oriented,30,25,.5,.03,.1,1,.3);
        FixedRouteAuto.Command parkCmd=wallPark.update(pose(0,24,40,0,0,0),0,0L,false,false);
        check(parkCmd.action!=FixedRouteAuto.Action.FAULT,"first waypoint at clearance is not blocked");
        parkCmd=wallPark.update(pose(.02,24,40,0,0,0),.02,0L,false,false);
        check(parkCmd.action==FixedRouteAuto.Action.PARK&&parkCmd.xPower<0,"oriented wall-adjacent park is feasible");
        check(oriented.pathSafe(24,40,0,9,40,0,9,9),"oriented park corridor stays in the wall rectangle");
        check(!oriented.pathSafe(24,40,0,8,40,0,9,9),"oriented corridor still refuses out-of-field park");
        check(!new SweptCaptureFeasibility(144,144,Math.hypot(9,9),null).pathSafe(24,40,0,9,40,0),"circle fallback still blocks wall park");
    }
    private static void basicDrive() {
        rejects(()->new BasicDrive.Settings(new String[]{"fl","bl","fr","br"},new boolean[4],.3,false),"unverified hardware cannot arm basic drive");
        rejects(()->new BasicDrive.Settings(new String[]{"fl","fl","fr","br"},new boolean[4],.3,true),"duplicate drive names refused");
        double[] forward=BasicDrive.powers(0,1,0,.3);for(double p:forward)near(p,.3,"forward wheel cap");
        double[] yaw=BasicDrive.powers(0,0,1,.3);near(yaw[0],-.3,"CCW left wheel reverse");near(yaw[2],.3,"CCW right wheel forward");
        double[] strafe=BasicDrive.powers(1,0,0,.3);near(strafe[0],.3,"right strafe FL");near(strafe[1],-.3,"right strafe BL");
        for(double p:BasicDrive.powers(1,1,1,.3))check(Math.abs(p)<=.3,"combined mix stays capped");
        check(BasicDrive.powers(Double.NaN,0,0,.3)==null&&BasicDrive.powers(2,0,0,.3)==null,"bad drive demands refused");
        DriverStickMap.Command robot=DriverStickMap.robotCentric(1,0,1);
        near(robot.fieldX,1,"robot-centric stick right is +X");near(robot.turn,-1,"stick right is clockwise");
        DriverStickMap.Command stickUp=DriverStickMap.robotCentric(0,-1,0);
        near(stickUp.fieldY,1,"robot-centric stick up is +Y");
        DriverStickMap.Command redUp=DriverStickMap.fieldCentric(0,-1,1,AllianceColor.RED);
        near(redUp.fieldX,1,"red stick up is away from the red wall (+X)");near(redUp.fieldY,0,"red stick up is not field +Y");
        near(redUp.turn,-1,"field-centric turn is also clockwise");
        DriverStickMap.Command redRight=DriverStickMap.fieldCentric(1,0,0,AllianceColor.RED);
        near(redRight.fieldX,0,"red stick right has no +X");near(redRight.fieldY,-1,"red stick right is driver-right (-Y)");
        DriverStickMap.Command blueUp=DriverStickMap.fieldCentric(0,-1,0,AllianceColor.BLUE);
        near(blueUp.fieldX,-1,"blue stick up is away from the blue wall (-X)");
        DriverStickMap.Command unknown=DriverStickMap.fieldCentric(0,-1,.5,AllianceColor.UNKNOWN);
        near(unknown.fieldX,0,"unknown alliance refuses field translation");near(unknown.fieldY,0,"unknown alliance zeroes field Y");
        near(unknown.turn,-.5,"unknown alliance still maps clockwise turn");
        DriverStickMap.Command rec=DriverStickMap.robotCentric(.2,.0,-.4);
        near(rec.turn,.4,"recovery/practice mapping matches Drive Practice sign");
    }
    private static final class RecoveryIO implements RobotIO {
        Snapshot sample;Command output=new Command();boolean allowed=true,driveHealthy=true,throwRead;int enters,normalReads,recoveryReads;
        public Snapshot read(double now,double dt) {normalReads++;if(throwRead)throw new IllegalStateException("sensor");return sample;}
        public void write(Command c) {output=c;}
        public void stop() {output=new Command();}
        public boolean enterDriveRecovery() {enters++;return allowed;}
        public Snapshot readDriveRecovery(double now,double dt) {recoveryReads++;return new Snapshot(RobotState.invalid(now),now,Double.NaN,Double.NaN,Double.NaN,
                new Health(driveHealthy,false,false,false,false),false,false,false,null,null,null,null);}
        public void writeDriveRecovery(Command c,double dt) {output=c;}
        void at(double t,Health health) {sample=new Snapshot(pose(t,60,50,0,0,0),t,0,0,1000,health,false,true,false,null,null,null,null);}
    }
    private static RobotCoordinator coordinator(RecoveryIO io,PieceInventory inventory) {
        return new RobotCoordinator(io,inventory,scorer(inventory),null,null,
                new TurretController(new TurretControlProfile(-2.9,2.9,-2.8,2.8,2.5,2,1,1,1,1,.1,.1,.3,1,2)),
                new DriveAssist(2,.3,.08),new HiveTracker(.1,.2),.02,.1,.1);
    }
    private static void recovery() {
        for(int failed=0;failed<4;failed++) {
            RecoveryIO io=new RecoveryIO();PieceInventory inventory=inventory();RobotCoordinator c=coordinator(io,inventory);
            io.at(0,new RobotIO.Health(true,failed!=0,failed!=1,failed!=2,failed!=3));
            RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=true;in.fieldX=.2;c.cycle(in,0,.02);
            check(io.output.enabled,"non-drive glitch keeps drive commandable");
            near(io.output.fieldX,.2,"drive demand retained through non-drive glitch");
            check(io.output.rpm==0&&io.output.feeder==0,"non-drive glitch still inhibits launcher");
            in.fieldX=0;in.recoverDrive=true;c.cycle(in,.02,.02);check(io.enters==0,"enabled recovery refused");
            in.enabled=false;in.fieldX=.1;c.cycle(in,.04,.02);check(io.enters==0,"moving-stick recovery refused");
            in.fieldX=0;c.cycle(in,.06,.02);check(c.driveRecovery(),"disabled neutral recovery accepted after subsystem fault");
            int reads=io.normalReads;in.recoverDrive=false;in.enabled=true;in.fieldX=.2;in.intake=in.shoot=in.preSpin=in.unwindAssist=true;
            c.cycle(in,.08,.02);check(io.normalReads==reads&&io.recoveryReads>0,"recovery never reads failed localization/mechanisms");
            near(io.output.fieldX,.2,"robot-centric manual demand retained");
            check(io.output.rpm==0&&io.output.feeder==0&&io.output.transfer==0&&io.output.intake==0&&io.output.turretPower==0
                    &&Double.isNaN(io.output.hood),"all mechanism commands suppressed in recovery");
            in.fieldX=Double.NaN;c.cycle(in,.09,.02);check(!io.output.enabled,"nonfinite recovery demand never publishes drive");in.fieldX=.2;
            in.enabled=false;c.cycle(in,.10,.02);check(!io.output.enabled,"recovery retains deadman");
            in.enabled=true;io.driveHealthy=false;c.cycle(in,.12,.02);check(!io.output.enabled,"drive fault prevents recovery outputs");
            if(failed==3)check(!inventory.isConfirmed(),"latched inventory sensor fault requires physical reconciliation");
        }
        RecoveryIO io=new RecoveryIO();RobotCoordinator c=coordinator(io,inventory());io.allowed=false;
        io.at(0,new RobotIO.Health(true,true,false,true,true));RobotCoordinator.Input in=new RobotCoordinator.Input();c.cycle(in,0,.02);
        in.recoverDrive=true;c.cycle(in,.02,.02);check(!c.driveRecovery(),"unconfigured or ambiguous-write recovery refused");
        debounce();
        bulkCache();
    }
    private static void debounce() {
        RecoveryIO io=new RecoveryIO();RobotCoordinator c=coordinator(io,inventory());
        RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=true;in.fieldX=.25;
        io.at(0,new RobotIO.Health(false,true,true,true,true));c.cycle(in,0,.02);
        check(!io.output.enabled,"one drive drop stops this cycle");
        io.at(.02,new RobotIO.Health(true,true,true,true,true));c.cycle(in,.02,.02);
        check(io.output.enabled,"single drive drop recovers without latch");near(io.output.fieldX,.25,"drive demand resumes");
        io.at(.04,new RobotIO.Health(false,true,true,true,true));c.cycle(in,.04,.02);
        io.at(.06,new RobotIO.Health(false,true,true,true,true));c.cycle(in,.06,.02);
        check(!io.output.enabled,"second consecutive drive drop latches");
        io.at(.08,new RobotIO.Health(true,true,true,true,true));c.cycle(in,.08,.02);
        check(!io.output.enabled,"drive latch stays fail-closed until recovery/reset");
        io=new RecoveryIO();c=coordinator(io,inventory());
        io.at(0,new RobotIO.Health(true,true,true,false,true));c.cycle(in,0,.02);
        check(io.output.enabled&&io.output.rpm==0,"one launcher drop keeps drive and zeros RPM");
        io.at(.02,new RobotIO.Health(true,true,true,true,true));c.cycle(in,.02,.02);
        check(io.output.enabled,"launcher glitch does not latch a global fault");
        io.at(.04,new RobotIO.Health(true,true,true,false,true));c.cycle(in,.04,.02);
        io.at(.06,new RobotIO.Health(true,true,true,false,true));c.cycle(in,.06,.02);
        check(io.output.enabled,"persistent launcher fault still leaves drive usable");
        in.enabled=false;in.fieldX=0;in.resetFault=true;c.cycle(in,.08,.02);
        in.resetFault=false;in.enabled=true;in.fieldX=.25;
        io.at(.10,new RobotIO.Health(true,true,true,true,true));c.cycle(in,.10,.02);
        check(io.output.enabled,"drive remains usable after launcher latch");
        HealthWatch watch=new HealthWatch();
        RobotIO.Health good=new RobotIO.Health(true,true,true,true,true);
        RobotIO.Health loc=new RobotIO.Health(true,false,true,true,true);
        HealthWatch.View view=watch.update(loc,true);
        check(view.driveCommandable&&!view.localizationTrusted&&!view.localizationLatched,"one loc miss degrades only localization");
        view=watch.update(good,true);
        check(view.localizationTrusted&&!view.localizationLatched,"one loc miss recovers");
        watch.update(loc,true);view=watch.update(loc,true);
        check(view.localizationLatched&&view.driveCommandable,"persistent loc latches without killing drive");
        watch.update(good,true);view=watch.update(good,true);
        check(!view.localizationLatched&&view.localizationTrusted,"non-drive latch clears after consecutive healthy samples");
        view=watch.update(good,false);
        check(!view.driveCommandable&&!view.timingLatched,"one long loop drops outputs this cycle");
        view=watch.update(good,true);
        check(view.driveCommandable&&!view.timingLatched,"single long loop does not latch");
        watch.update(good,false);view=watch.update(good,false);
        check(view.timingLatched&&!view.driveCommandable,"persistent overruns latch timing fail-closed");
        check(RobotIO.Health.yawHealthy(true,false,false),"live analog and no trip is yaw-healthy");
        check(!RobotIO.Health.yawHealthy(false,false,false),"analog disconnect is not yaw-healthy");
        check(!RobotIO.Health.yawHealthy(true,true,false),"wrong-direction trip is not yaw-healthy");
        check(!RobotIO.Health.yawHealthy(true,false,true),"turret write fault is not yaw-healthy");
        io=new RecoveryIO();c=coordinator(io,inventory());
        in=new RobotCoordinator.Input();in.enabled=true;in.fieldX=.25;
        io.at(0,new RobotIO.Health(true,true,RobotIO.Health.yawHealthy(false,false,false),true,true));
        c.cycle(in,0,.02);
        check(io.output.enabled&&io.output.fieldX==.25&&io.output.turretPower==0,
                "analog disconnect degrades yaw and keeps drive");
        io.at(.02,new RobotIO.Health(true,true,RobotIO.Health.yawHealthy(true,true,false),true,true));
        c.cycle(in,.02,.02);
        check(io.output.enabled&&io.output.turretPower==0,"wrong-direction trip degrades yaw and keeps drive");
    }
    private static final class FakeHub implements HubBulkCache.Hub {
        int manuals,clears;
        public void setManual() {manuals++;}
        public void clear() {clears++;}
    }
    private static void bulkCache() {
        FakeHub a=new FakeHub(),b=new FakeHub();
        HubBulkCache cache=new HubBulkCache(Arrays.asList(a,b));
        boolean refused=false;try {cache.beginLoop();}catch(IllegalStateException expected){refused=true;}
        check(refused,"MANUAL cache cannot clear before enable");
        cache.enableManual();check(a.manuals==1&&b.manuals==1,"every hub is set to MANUAL");
        cache.beginLoop();check(a.clears==1&&b.clears==1&&cache.clears()==1,"one clear per loop on every hub");
        cache.beginLoop();check(a.clears==2&&cache.clears()==2,"clears accumulate once per beginLoop");
        boolean nullHub=false;try {new HubBulkCache(Arrays.asList(a,(HubBulkCache.Hub)null));}catch(IllegalArgumentException expected){nullHub=true;}
        check(nullHub,"null hub refused");
    }
    private static CalibratedShotTable table() {
        LauncherGeometry.Release[][] grid={{new LauncherGeometry.Release(10,0,12,.2),new LauncherGeometry.Release(10,0,12,.2)},
                {new LauncherGeometry.Release(10,0,12,.6),new LauncherGeometry.Release(10,0,12,.6)}};
        LauncherGeometry g=new LauncherGeometry(new double[]{.2,.6},new double[]{30,50},grid,
                new CalibratedAxis(new double[]{.2,.6},new double[]{.2,.8}),new CalibratedAxis(new double[]{30,50},new double[]{.2,.8}),.05);
        List<CalibratedShotTable.Point> points=new ArrayList<CalibratedShotTable.Point>();
        for(BallType type:BallType.values())for(int i=0;i<2;i++)points.add(new CalibratedShotTable.Point(20+120*i,
                new ShotRecipe(type,1000,i==0?.2:.6,type==BallType.POLLEN?30:50,g,.05,40,.05,20,140),.5,.01,.05,.005,35,.25));
        return new CalibratedShotTable(points);
    }
    private static void stationary() {
        CalibratedShotTable table=table();MotionShotSolver.StationarySettings s=new MotionShotSolver.StationarySettings(.05,.05,.05,2,.05);
        RobotState r=pose(0,60,50,0,0,0);
        MotionShotSolver.Solution solution=MotionShotSolver.stationary(table,BallType.POLLEN,r,0,0,0,0,0,150,50,35,.1,.01,s);
        check(solution!=null&&solution.confident,"stationary solution needs no moving calibration");near(solution.equivalentRange,80,"range measured from release point not pivot");
        near(solution.releaseX,70,"pitch/gap geometry release offset");near(solution.bearing,0,"stationary bearing");
        check(MotionShotSolver.stationary(table,BallType.POLLEN,r,0,0,0,0,0,210,50,35,.1,.01,s)!=null,
                "valid outer muzzle range accepted even when pivot lies beyond table limit");
        check(MotionShotSolver.stationary(table,BallType.POLLEN,pose(0,60,50,0,0,1),0,0,0,0,0,150,50,35,.1,.01,s)==null,"moving robot refused");
        check(MotionShotSolver.stationary(table,BallType.POLLEN,r,0,0,0,0,.1,150,50,35,.1,.01,s)==null,"moving turret refused");
        check(MotionShotSolver.stationary(table,BallType.POLLEN,r,0,0,0,0,0,150,50,40,.1,.01,s)==null,"uncalibrated height refused");
        check(MotionShotSolver.stationary(table,BallType.POLLEN,r,0,0,0,0,0,260,50,35,.1,.01,s)==null,"out-of-range shot refused");
        RecoveryIO io=new RecoveryIO();PieceInventory p=inventory();ScoringController scorer=new ScoringController(p,table,
                new ScoringController.Timing(1,.15,.3,.1,1,.2,0),.3,.2,null);
        scorer.setFlightTiming(.02,.1);scorer.setLauncherLimits(new LauncherFeedback.Limits(.01,.02,.5,1,.05,.1));
        RobotCoordinator c=new RobotCoordinator(io,p,scorer,null,null,new TurretController(new TurretControlProfile(-2.9,2.9,-2.8,2.8,2.5,2,1,1,1,1,.1,.1,.3,1,2)),
                new DriveAssist(2,.3,.08),new HiveTracker(.05,.2),.02,.1,.01);c.setStationaryShots(s);
        RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=true;in.shotRequestId=1;
        for(int i=0;i<=8;i++) {
            double t=i*.02;
            io.sample=new RobotIO.Snapshot(pose(t,60,50,0,0,0),t,0,0,1000,true,false,true,false,null,
                    new TargetEstimate(150,50,35,t,1,0,HiveTracker.State.SIDE_A_UP,true,2),null,
                    new LauncherFeedback(t,.4,0,30,0));c.cycle(in,t,.02);
        }
        check(scorer.state()==ScoringController.State.FEED_ONE,"stationary runtime reaches measured feed without tracker/planner/moving settings");
        near(io.output.feeder,.3,"stationary measured feed output");
        in.mechanismsInhibited=true;c.cycle(in,.18,.02);
        check(io.output.feeder==0&&io.output.turretPower==0&&io.output.rpm==0,"departure inhibition independently suppresses mechanisms");
    }
    private static void outcomes() {
        ShotJournal log=new ShotJournal("SYNTHETIC","run-1");RobotIO.Command command=new RobotIO.Command();command.feeder=.3;
        RobotIO.Snapshot sample=new RobotIO.Snapshot(pose(0,60,50,0,0,0),0,0,0,1000,true,false,true,false,null,null,null);
        log.record(0,ScoringController.State.FEED_ONE,sample,command,null,null,0,0,0,0,0);
        check(log.lastConfirmedShotId()==0,"feeder start cannot be labeled as confirmed exit");
        sample=new RobotIO.Snapshot(pose(.1,60,50,0,0,0),.1,0,0,1000,true,false,true,true,null,null,null);
        log.record(.1,ScoringController.State.WAIT_FOR_HIVE,sample,new RobotIO.Command(),null,null,0,0,0,0,0);
        check(log.lastConfirmedShotId()==1,"outcome refers to last confirmed release");
        log.outcome(1,.2,"HIT",null);String row,last=null;while((row=log.poll())!=null)last=row;
        check(last!=null&&last.contains("OUTCOME")&&last.contains("HIT")&&last.contains("run-1"),"observer outcome contains shot and run identity");
        check(last.split(",",-1).length==ShotJournal.HEADER.split(",").length,"outcome row matches extended schema");
    }
    private static void multiShotHandoff() {
        RecoveryIO io=new RecoveryIO();PieceInventory p=inventory();RobotCoordinator robot=coordinator(io,p);
        SweptCaptureFeasibility bounds=new SweptCaptureFeasibility(144,144,5,null);
        FixedRouteAuto route=new FixedRouteAuto(new FixedRouteAuto.Waypoint[]{
                new FixedRouteAuto.Waypoint(40,40,0,8,true),
                new FixedRouteAuto.Waypoint(55,40,0,8,false,true),
                new FixedRouteAuto.Waypoint(70,40,0,8,true)},
                new FixedRouteAuto.Waypoint(80,40,0,5,false),bounds,30,25,1,.05,.05,1,.3);
        RobotCoordinator.Input in=new RobotCoordinator.Input();
        int previous=p.controlledCount(),shots=0;boolean collected=false;
        double x=40,y=40;
        for(int step=0;step<120;step++) {
            double t=step*.02;
            boolean exit=robot.scoring.state()==ScoringController.State.FEED_ONE
                    ||robot.scoring.state()==ScoringController.State.CONFIRM_EXIT;
            boolean entry=in.intake&&!collected&&Math.abs(x-55)<=1;
            io.sample=new RobotIO.Snapshot(pose(t,x,y,0,0,0),t,0,0,1000,true,entry,true,exit,BallType.POLLEN,
                    new TargetEstimate(x+80,y,t,1,0,HiveTracker.State.SIDE_A_UP),null);
            boolean pickup=p.controlledCount()>previous;previous=p.controlledCount();
            if(robot.snapshot==null) {in.enabled=true;robot.cycle(in,t,.02);continue;}
            FixedRouteAuto.Command cmd=route.update(robot.snapshot.robot,t,robot.scoring.completedRequestId(),pickup,
                    robot.scoring.state()==ScoringController.State.FAULT);
            check(cmd.action!=FixedRouteAuto.Action.FAULT,"multi-shot route never faults: "+cmd.reason+" scoring="+robot.scoring.state());
            in.enabled=cmd.action!=FixedRouteAuto.Action.DONE;in.fieldX=cmd.xPower;in.fieldY=cmd.yPower;in.turn=cmd.turnPower;
            in.shotRequestId=route.shotRequestId();in.cancelShot=cmd.action!=FixedRouteAuto.Action.SHOOT;
            in.intake=route.intakeRequested();in.mechanismsInhibited=route.departing();
            robot.cycle(in,t,.02);
            if(robot.scoring.completedRequestId()>shots)shots=(int)robot.scoring.completedRequestId();
            if(pickup)collected=true;
            if(Math.abs(cmd.xPower)>1e-6)x+=Math.signum(cmd.xPower)*Math.min(1,Math.abs(cmd.xPower)*20);
            if(cmd.action==FixedRouteAuto.Action.DONE) {
                check(shots>=2,"two confirmed shots before park");
                check(collected,"collect leg ran between shots");
                check(p.controlledCount()>=1,"second shot left remaining inventory without scoring fault");
                return;
            }
        }
        check(false,"multi-shot route should park after two shots scoring="+robot.scoring.state()+" shots="+shots);
    }
    public static void main(String[] args) {actions();departures();basicDrive();recovery();stationary();outcomes();multiShotHandoff();System.out.println("COMPETITION READINESS CHECKS PASSED ("+checks+")");}
}
