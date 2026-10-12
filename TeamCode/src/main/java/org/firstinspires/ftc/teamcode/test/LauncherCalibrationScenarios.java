package org.firstinspires.ftc.teamcode.test;

import java.io.*;
import java.util.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;

/** Synthetic measurements: validates implementation and failure gates, never physical accuracy. */
public final class LauncherCalibrationScenarios {
    private static int checks;
    private static void check(boolean pass,String reason) {checks++;if(!pass)throw new AssertionError(reason);}
    private static void near(double actual,double expected,String reason) {check(Math.abs(actual-expected)<.002,reason+": "+actual);}
    private static void rejects(Runnable action,String reason) {
        boolean rejected=false;try {action.run();}catch(IllegalArgumentException expected) {rejected=true;}check(rejected,reason);
    }
    private static LauncherGeometry geometry() {
        LauncherGeometry.Release[][] grid=new LauncherGeometry.Release[2][2];
        for(int i=0;i<2;i++)for(int j=0;j<2;j++)grid[i][j]=new LauncherGeometry.Release(2+4*i+2*j,j,10+3*i+j,.2+.4*i);
        return new LauncherGeometry(new double[]{.2,.6},new double[]{30,50},grid,
                new CalibratedAxis(new double[]{.2,.4,.6},new double[]{.1,.3,.9}),
                new CalibratedAxis(new double[]{30,50},new double[]{.2,.8}),.1);
    }
    private static CalibratedShotTable table() {
        LauncherGeometry g=geometry();List<CalibratedShotTable.Point> points=new ArrayList<CalibratedShotTable.Point>();
        for(BallType type:BallType.values())for(int i=0;i<2;i++) {
            ShotRecipe recipe=new ShotRecipe(type,1000+400*i,i==0?.2:.6,type==BallType.POLLEN?30:50,g,.05,40,.05,20,140);
            points.add(new CalibratedShotTable.Point(20+120*i,recipe,.5,.01,.05,.005,35,.25));
        }
        return new CalibratedShotTable(points);
    }
    private static LauncherFeedback.Limits limits() {return new LauncherFeedback.Limits(.01,.02,.5,1,.05,.1);}
    private static PieceInventory inventory() {PieceInventory p=new PieceInventory(4);p.reconcile(Arrays.asList(BallType.POLLEN,BallType.POLLEN));return p;}
    private static ScoringController scorer(BurstPolicy burst) {
        ScoringController c=new ScoringController(inventory(),table(),new ScoringController.Timing(2,.15,.3,.1,1,.2,0),.3,.2,burst);
        c.setFlightTiming(.02,.1);c.setLauncherLimits(limits());return c;
    }
    private static ScoringController.Input input(double t) {
        ScoringController.Input in=new ScoringController.Input();in.nowSec=in.acquiredSec=t;
        in.healthy=in.enabled=in.staged=in.aimReady=in.targetFresh=in.stationary=in.hiveStable=true;
        in.hiveObservedSec=in.targetObservedSec=t;in.rangeInches=80;in.targetHeight=35;
        in.rpm=1200;in.hiveState=HiveTracker.State.SIDE_A_UP;
        in.launcher=new LauncherFeedback(t,.4,0,30,0);
        in.impact=new TargetEstimate(80,0,35,t,1,0,in.hiveState,true,3);
        in.carried=inventory().load();return in;
    }
    private static void axesAndTables() throws IOException {
        AxonCrCalibration cr=new AxonCrCalibration(500,1520,2500,20000,-1);
        near(cr.sdkPower(0),.02,"verified neutral is not assumed SDK zero");
        near(cr.sdkPower(1),-1,"calibrated reverse sign");near(cr.sdkPower(-1),1,"full positive endpoint");
        rejects(()->new AxonCrCalibration(500,2500,2500,20000,1),"bad neutral rejected");
        rejects(()->cr.sdkPower(Double.NaN),"invalid CR demand rejected");
        LauncherGeometry g=geometry();LauncherGeometry.Release middle=g.at(.4,40);
        near(middle.forward,5,"bilinear release forward");near(middle.height,12,"pitch/gap release height");
        near(middle.left,.5,"gap release lateral offset");check(g.at(.61,40)==null,"geometry never extrapolates");
        ShotRecipe r=table().select(BallType.POLLEN,80).recipe;
        near(r.pitchRad,.4,"interpolate physical pitch");near(r.hood,.3,"apply nonlinear servo map after interpolation");
        near(r.gapMm,30,"pollen compression fixed");near(table().select(BallType.NECTAR,80).recipe.gapMm,50,"nectar compression separate");
        String csv=ShotCalibrationCsv.HEADER+"\n20,1000,15,30,.05,40,.05,.5,.01,.04,.003,35,.25\n140,1400,30,30,.05,40,.05,.6,.02,.06,.004,35,.25\n";
        CalibratedShotTable loaded=ShotCalibrationCsv.load(new StringReader(csv),new StringReader(csv.replace(",30,.05",",50,.05")),g);
        near(loaded.select(BallType.NECTAR,80).recipe.gapMm,50,"separate measured CSV types");
        near(loaded.select(BallType.POLLEN,80).releaseDelaySec,.05,"release delay interpolated independently");
        rejects(()->{try {ShotCalibrationCsv.load(new StringReader(ShotCalibrationCsv.HEADER),new StringReader(csv),g);}catch(IOException e){throw new AssertionError(e);}},"empty measurement template cannot arm");
        ShotRecipe otherGap=new ShotRecipe(BallType.POLLEN,1400,.6,50,g,.05,40,.05,20,140);
        rejects(()->new CalibratedShotTable(Arrays.asList(new CalibratedShotTable.Point(20,
                new ShotRecipe(BallType.POLLEN,1000,.2,30,g,.05,40,.05,20,140),.5,.01,.05,.005,35,.25),
                new CalibratedShotTable.Point(140,otherGap,.5,.01,.05,.005,35,.25))),"distance-dependent initial compression rejected");
    }
    private static void achievedReadiness() {
        for(int condition=0;condition<6;condition++) {
            ScoringController c=scorer(null);ScoringController.Input in=input(0);in.start=true;c.update(in);
            in=input(.08);
            if(condition==0)in.launcher=new LauncherFeedback(.08,.43,0,30,0);
            if(condition==1)in.launcher=new LauncherFeedback(.08,.4,.03,30,0);
            if(condition==2)in.launcher=new LauncherFeedback(.08,.4,0,32,0);
            if(condition==3)in.launcher=new LauncherFeedback(.08,.4,0,30,2);
            if(condition==4)in.launcher=null;
            if(condition==5)in.launcher=new LauncherFeedback(0,.4,0,30,0);
            check(c.update(in).feeder==0,"unachieved or repeated feedback cannot feed: "+condition);
            check(c.state()==ScoringController.State.PREPARE,"remains preparing: "+condition);
        }
        ScoringController c=scorer(null);ScoringController.Input in=input(0);in.start=true;c.update(in);
        check(c.update(input(.08)).feeder>0,"achieved measured pitch/gap/RPM permit feed");
        in=input(.09);in.launcher=new LauncherFeedback(.09,.45,0,30,0);
        check(c.update(in).feeder==0&&c.state()==ScoringController.State.FAULT,"load deflection stops release and faults inventory");
        c=scorer(null);in=input(0);in.preSpin=true;
        ScoringController.Output output=c.update(in);check(output.rpm>0&&output.feeder==0&&output.transfer==0,"deliberate pre-spin has no feed outputs");
        in=input(.06);in.preSpin=true;c.update(in);in=input(.07);in.start=true;
        check(c.update(in).feeder>0,"measured pre-spin readiness is retained on start");
        c=scorer(null);in=input(0);in.preSpin=true;in.enabled=false;check(c.update(in).rpm==0,"enable gates pre-spin");
        c=scorer(null);in=input(0);in.start=true;in.targetHeight=40;
        check(c.update(in).rpm==0,"wrong target height cannot select empirical table");
        BurstPolicy unvalidated=new BurstPolicy(1,.1,.1,.05,.005,null);
        c=scorer(unvalidated);in=input(0);in.start=in.burst=true;
        check(c.update(in).rpm==0&&c.state()==ScoringController.State.IDLE,"unvalidated physical burst inhibited");
        c=scorer(unvalidated);in=input(0);in.start=true;c.update(in);
        check(c.update(input(.08)).feeder>0,"ordinary scoring remains available with unvalidated burst configured");
    }
    private static void solverAndCell() {
        RobotState r=new RobotState(0,0,0,0,0,0,0,0,0,0,true,true,0,true,0);
        MotionConfidence.Limits motion=new MotionConfidence.Limits(.2,40,3,30,3,.5,.03,.1,.1,.1,.001,.01);
        MotionShotSolver solver=new MotionShotSolver(new MotionShotSolver.Settings(.06,.006,0,0,.2,.3,30,3,10,.01,motion));
        MotionShotSolver.Solution result=solver.solve(table(),BallType.POLLEN,r,0,.06,0,0,0,0,0,0,0,80,0,35,true,false);
        check(result!=null,"state-dependent geometry solver converges");
        near(result.releaseX,result.selection.recipe.geometry.at(result.selection.recipe.pitchRad,30).forward,"solver uses selected pitch release offset");
        near(result.releaseHeight,result.selection.recipe.geometry.at(result.selection.recipe.pitchRad,30).height,"solver preserves actual release height");
        near(result.releaseSec,.05,"solver uses per-shot measured release delay");
        check(solver.solve(table(),BallType.POLLEN,r,0,.06,0,0,0,0,0,0,0,80,0,40,true,false)==null,"empirical height mismatch inhibits solver");
        CellTargetGeometry cells=new CellTargetGeometry(Arrays.asList(new CellTargetGeometry.Point(1,HiveTracker.State.SIDE_A_UP,2,0,1,3)));
        double[] offset=cells.offset(1,HiveTracker.State.SIDE_A_UP,CellTargetGeometry.rotation(Math.PI/2,0,0));
        near(offset[0],0,"observed tag rotation applied x");near(offset[1],2,"chosen cell point rotated y");near(offset[2],1,"chosen cell point preserves height");
        check(cells.offset(1,HiveTracker.State.UNKNOWN,CellTargetGeometry.rotation(0,0,0))==null,"unknown hive has no invented cell point");
        RobotStateHistory history=new RobotStateHistory();history.add(r);TurretPoseHistory yaw=new TurretPoseHistory();yaw.add(0,0,0);
        CameraTargetTransform transform=new CameraTargetTransform(new double[]{1,0,0,0,1,0,0,0,1},0,0,0,0,10,1,200,cells);
        TargetEstimate target=transform.project(80,0,0,CellTargetGeometry.rotation(Math.PI/2,0,0),0,1,HiveTracker.State.SIDE_A_UP,history,yaw,0);
        check(target!=null&&target.internalPoint,"camera projects measured cell point");near(target.y,2,"camera tag-to-cell lateral offset");near(target.z,11,"camera and cell heights combined");
    }
    private static void journal() {
        ShotJournal log=new ShotJournal("SYNTHETIC");ScoringController.Input in=input(0);
        RobotState r=new RobotState(0,0,0,0,0,0,0,0,0,0,true,true,0,true,0);
        RobotIO.Snapshot s=new RobotIO.Snapshot(r,0,0,0,1200,true,false,true,false,null,in.impact,null,in.launcher);
        RobotIO.Command c=new RobotIO.Command();c.rpm=1200;c.feeder=.3;c.pitchRad=.4;c.gapMm=30;
        log.record(0,ScoringController.State.FEED_ONE,s,c,table().select(BallType.POLLEN,80).recipe,null,0,0,0,0,1);
        String row=log.poll();check(log.currentShotId()==1&&row.contains("FEED_START"),"feeder start produces shot ID and record");
        check(row.split(",",-1).length==ShotJournal.HEADER.split(",").length,"shot CSV fields match header");
        log.outcome(1,.8,"SCORED","NO TIP");row=log.poll();check(row.contains("OUTCOME"),"observed outcome attached by shot ID");
        check(row.split(",",-1).length==ShotJournal.HEADER.split(",").length,"outcome shares shot CSV schema");
        rejects(()->log.outcome(2,.8,"SCORED","TIP"),"unknown shot outcome rejected");
        for(int i=1;i<520;i++)log.record(i*.1,ScoringController.State.FEED_ONE,s,c,null,null,0,0,0,0,1);
        check(log.dropped>0,"bounded log reports overflow");
    }
    private static final class FakeIO implements RobotIO {
        Command command=new Command();
        public Snapshot read(double now,double dt) {
            RobotState r=new RobotState(now,60,50,0,0,0,0,0,0,0,true,true,now,true,0);
            return new Snapshot(r,now,0,0,1000,true,false,true,false,null,
                    new TargetEstimate(100,50,now,1,0,HiveTracker.State.SIDE_A_UP),null);
        }
        public void write(Command command) {this.command=command;}
        public void stop() {command=new Command();}
    }
    private static void intakeIsolation() {
        for(boolean isolated:new boolean[]{false,true}) {
            FakeIO io=new FakeIO();PieceInventory inventory=inventory();
            ScoringController scoring=new ScoringController(inventory,Arrays.asList(
                    new ShotRecipe(BallType.POLLEN,1000,.4,.6,.1,40,.1)),new ScoringController.Timing(2,.1,.2,.1,1,.2,0));
            RobotCoordinator robot=new RobotCoordinator(io,inventory,scoring,
                    new PieceTracker(HomographyFloorProjection.fromCoefficients(new double[]{1,0,0,0,1,0,0,0,1}),PieceTrackerConstants.defaults()),
                    new PickupPlanner(new MapTipModel().set(2,0,1),new EuclideanTravelTimeModel(),new FixedShotSetupModel(1)),
                    new TurretController(),new DriveAssist(2,.3,.08),new HiveTracker(.1,.2),.02,.1,.1);
            robot.setFeedStagesIsolated(isolated);
            RobotCoordinator.Input in=new RobotCoordinator.Input();in.enabled=in.intake=in.shoot=true;
            robot.cycle(in,0,.02);
            check(scoring.state()==ScoringController.State.PREPARE,"preparing during isolation scenario");
            check((io.command.intake>0)==isolated,"concurrent intake requires measured feed isolation");
            in.enabled=false;robot.cycle(in,.02,.02);check(io.command.intake==0,"enable gates isolated intake");
        }
    }
    private static void physicalBursts() {
        BurstPolicy.ImpactModel countOnly=(type,id,hive,observed,min,max,pending)->true;
        ScoringController c=scorer(new BurstPolicy(2,.1,.1,.05,.005,countOnly,true,true));
        ScoringController.Input in=input(0);in.start=in.burst=true;c.update(in);
        in=input(.08);in.start=in.burst=true;
        check(c.update(in).feeder==0,"legacy count-only impact callback cannot certify physical burst");
        final int[] calls={0};
        BurstPolicy.ImpactModel measured=new BurstPolicy.ImpactModel() {
            public boolean safe(BallType type,int id,HiveTracker.State state,double observed,double min,double max,List<BurstPolicy.Flight> pending) {
                throw new AssertionError("Physical shot must pass complete context");
            }
            public boolean safe(ShotTipContext context) {
                calls[0]++;return context.shot.recipe.physical()&&context.impact.internalPoint
                        &&context.arrivalMaxSec>context.arrivalMinSec&&context.cadenceSec>0;
            }
        };
        c=scorer(new BurstPolicy(2,.1,.1,.05,.005,measured,true,true));in=input(0);in.start=in.burst=true;c.update(in);
        in=input(.08);in.start=in.burst=true;check(c.update(in).feeder>0&&calls[0]>0,"physical impact/cadence context reaches measured model");
        for(BallType type:BallType.values()) {
            PieceInventory pieces=new PieceInventory(4);pieces.reconcile(Arrays.asList(type,type,type,type));
            c=new ScoringController(pieces,table(),new ScoringController.Timing(2,.15,.3,.1,1,.2,0),.3,.2,
                    new BurstPolicy(1,.1,.1,.05,.005,null,true,true));
            c.setFlightTiming(.02,.1);c.setLauncherLimits(limits());
            double feederStarted=Double.NaN;int exits=0;
            for(int step=0;step<400;step++) {
                double now=step*.01;in=input(now);in.start=in.burst=true;
                in.carried=pieces.load();in.launcher=new LauncherFeedback(now,.4,0,type==BallType.POLLEN?30:50,0);
                if(Double.isFinite(feederStarted)&&now-feederStarted>=.05-1e-9) {in.exit=true;feederStarted=Double.NaN;exits++;}
                ScoringController.State before=c.state();ScoringController.Output output=c.update(in);
                if(before!=ScoringController.State.FEED_ONE&&c.state()==ScoringController.State.FEED_ONE&&output.feeder>0)feederStarted=now;
                check(c.state()!=ScoringController.State.FAULT,"accepted burst trial retains known state: "+type+" step "+step);
            }
            check(exits==(type==BallType.NECTAR?3:4),"separate rapid trial size bound: "+type);
            check(pieces.pieces().size()==(type==BallType.NECTAR?1:0),"held trigger never exceeds rapid trial: "+type);
        }
    }
    public static void main(String[] args) throws IOException {
        axesAndTables();achievedReadiness();solverAndCell();journal();intakeIsolation();physicalBursts();
        System.out.println("LAUNCHER CALIBRATION CHECKS PASSED ("+checks+")");
    }
}
