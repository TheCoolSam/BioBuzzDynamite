package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.match.AllianceColor;
import java.util.*;

/** Integrated calibrated scoring TeleOp. Remains powerless until measured settings exist. */
@TeleOp(name="BIOBUZZ TeleOp",group="Competition")
public class BioBuzzTeleOp extends OpMode {
    protected RobotHardwareConfig.Settings settings;
    protected RevRobotIO io;
    protected RobotCoordinator robot;
    protected long epochNanos,lastNanos;
    protected String configurationFault="Hardware measurements are undecided";
    private double lastLog;
    private PickupAssist pickup;
    private boolean previousPickupHold;
    private String pickupReason="";
    private final List<BallType> recoveryDraft=new ArrayList<BallType>();
    private boolean previousPollen,previousNectar,previousUndo,previousClear,previousCommit;
    private final LoopMetrics metrics=new LoopMetrics();
    private final LoopMetrics intervals=new LoopMetrics();
    private boolean previousHit,previousMiss,previousTip,previousNoTip;
    protected RobotHardwareConfig.Settings loadSettings() {return RobotHardwareConfig.measured();}
    protected boolean stationaryProfileRequired() {return false;}
    @Override public void init() {
        epochNanos=lastNanos=System.nanoTime();telemetry.setMsTransmissionInterval(100);
        settings=loadSettings();
        if(settings!=null)try {
            if(stationaryProfileRequired()&&(settings.stationaryShots==null||settings.movingShots!=null||settings.burst!=null))
                throw new IllegalStateException("Stationary profile requires stationary bounds and no moving/burst mode");
            settings.validate();io=new RevRobotIO(hardwareMap,settings,epochNanos);
            PieceInventory inventory=new PieceInventory(4);inventory.reconcile(settings.confirmedPreload);
            ScoringController scoring=settings.shotTable==null
                    ?new ScoringController(inventory,settings.recipes,settings.scoringTiming,settings.feedPower,settings.reversePower)
                    :new ScoringController(inventory,settings.shotTable,settings.scoringTiming,settings.feedPower,settings.reversePower,settings.burst);
            if(settings.shotTable!=null)scoring.setRetargetTolerance(settings.rpmRetargetFraction,settings.hoodRetargetTolerance,settings.compressionRetargetTolerance);
            if(settings.shotTable!=null)scoring.setFlightTiming(settings.exitLatencySec,settings.impactSettleSec);
            if(Double.isFinite(settings.flywheelHoldRpm))scoring.setHoldRpm(settings.flywheelHoldRpm);
            robot=new RobotCoordinator(io,inventory,scoring,settings.pickupPlanningEnabled?new PieceTracker(settings.floor,settings.pieces):null,
                    settings.pickupPlanningEnabled?new PickupPlanner(settings.tipModel,settings.travel,settings.shots,settings.footprint):null,
                    new TurretController(settings.turretProfile),new DriveAssist(settings.yawAtFullPower,.3,.08),
                    new HiveTracker(settings.hiveSettleSec,settings.hiveMaxAgeSec),settings.aimTolerance,settings.maxMotion,settings.maxTurretRate);
            robot.setShotLogging(settings.calibrationRevision,Long.toHexString(System.currentTimeMillis())+"-"+Long.toHexString(epochNanos));robot.setShotTipModel(settings.tipModel);
            RobotLog.ii("BioBuzzShot","%s",ShotJournal.HEADER);
            scoring.setLauncherLimits(settings.launcherLimits);
            robot.setFeedStagesIsolated(settings.feedStagesIsolated);
            robot.setIntakePower(settings.intakePower);
            robot.setTurretPivot(settings.camera.pivotForward(),settings.camera.pivotLeft());
            if(settings.shotTable==null)robot.setScoringPoses(settings.shots,settings.scoringPositionTolerance,settings.scoringHeadingTolerance);
            if(settings.movingShots!=null)robot.setMovingShots(settings.movingShots);
            else robot.setStationaryShots(settings.stationaryShots);
            configurationFault="";
        }catch(RuntimeException failure) {configurationFault=failure.getMessage();if(io!=null)io.close();io=null;robot=null;}
        display();
    }
    @Override public void init_loop() {
        if(io!=null)try {
            long n=System.nanoTime();io.read(seconds(n),(n-lastNanos)*1e-9);lastNanos=n;io.stop();
        }catch(RuntimeException failure){configurationFault=failure.getMessage();io.close();io=null;robot=null;}
        display();
    }
    @Override public void start() {lastNanos=System.nanoTime();if(io!=null)io.stop();}
    protected double seconds(long nanos) {return(nanos-epochNanos)*1e-9;}
    protected RobotCoordinator.Input input() {
        RobotCoordinator.Input input=new RobotCoordinator.Input();
        input.enabled=gamepad1.right_bumper;
        input.cancel=gamepad1.b;input.resetFault=gamepad1.back&&gamepad1.start;
        input.intake=gamepad1.left_trigger>.5;input.shoot=gamepad1.a;
        input.burst=settings!=null&&settings.burst!=null&&gamepad2.right_bumper;
        input.preSpin=gamepad2.left_bumper;
        input.unwindAssist=gamepad1.left_bumper;
        DriverStickMap.Command sticks=robot!=null&&robot.driveRecovery()
                ?DriverStickMap.robotCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x)
                :DriverStickMap.fieldCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,
                    settings==null?AllianceColor.UNKNOWN:settings.alliance);
        input.fieldX=sticks.fieldX;input.fieldY=sticks.fieldY;input.turn=sticks.turn;
        boolean hold=gamepad1.x;
        boolean neutral=input.fieldX==0&&input.fieldY==0&&input.turn==0;
        input.resetFault &= neutral;
        input.recoverDrive=gamepad1.back&&gamepad1.y&&neutral&&!input.enabled;
        if(!hold||!input.enabled||input.cancel||!neutral||input.shoot||robot==null||!robot.inventory.isConfirmed()
                ||robot.driveRecovery()||robot.scoring.state()!=ScoringController.State.IDLE)pickup=null;
        else if(hold&&!previousPickupHold&&settings.pickupMotion!=null&&robot.snapshot!=null) {
            try {pickup=new PickupAssist(settings.pickupMotion,robot.plan,robot.snapshot.robot,
                    seconds(System.nanoTime()),robot.inventory.pieces().size());}
            catch(IllegalArgumentException refused) {pickupReason=refused.getMessage();}
        }
        previousPickupHold=hold;
        if(pickup!=null) {
            PickupAssist.Command move=pickup.update(robot.snapshot.robot,seconds(System.nanoTime()),robot.inventory.pieces().size());
            input.fieldX=move.x;input.fieldY=move.y;input.turn=move.turn;input.intake=move.intake;
            input.expectedPickupType=pickup.expectedType;input.unwindAssist=false;pickupReason=move.reason;
        }
        return input;
    }
    @Override public void loop() {
        long now=System.nanoTime();double dt=(now-lastNanos)*1e-9;lastNanos=now;
        intervals.record(dt);
        if(robot!=null) {
            RobotCoordinator.Input command=input();
            recoverInventory(command);
            robot.cycle(command,seconds(now),dt);
            annotateOutcome(seconds(now));
        }
        if(robot!=null&&robot.journal!=null) {
            String record;int count=0;
            while(count++<8&&(record=robot.journal.poll())!=null)RobotLog.ii("BioBuzzShot","%s",record);
        }
        if(robot!=null&&seconds(now)-lastLog>=.5) {
            double[] timing=metrics.summary();
            double[] cadence=intervals.summary();
            RobotLog.ii("BioBuzz","t=%.3f scoring=%s inventory=%s inhibit=%s loopP50=%.4f loopP95=%.4f loopP99=%.4f max=%.4f intervalP99=%.4f intervalMax=%.4f read=%.4f plan=%.4f write=%.4f cycle=%.4f",
                    seconds(now),robot.scoring.state(),robot.inventory.load(),robot.reason,timing[0],timing[1],timing[2],timing[3],
                    cadence[2],cadence[3],robot.readSeconds,robot.planSeconds,robot.writeSeconds,robot.cycleSeconds);lastLog=seconds(now);
        }
        display();
        metrics.record((System.nanoTime()-now)*1e-9); // Includes logging and telemetry construction.
    }
    private void annotateOutcome(double now) {
        boolean hit=gamepad2.a,miss=gamepad2.b,tip=gamepad2.left_stick_button,noTip=gamepad2.right_stick_button;
        if(robot.journal!=null) {
            long id=robot.journal.lastConfirmedShotId();
            if(id>0) {
                if(hit&&!miss&&!previousHit)robot.journal.outcome(id,now,"HIT",null);
                if(miss&&!hit&&!previousMiss)robot.journal.outcome(id,now,"MISS",null);
                if(tip&&!noTip&&!previousTip)robot.journal.outcome(id,now,null,"TIP");
                if(noTip&&!tip&&!previousNoTip)robot.journal.outcome(id,now,null,"NO_TIP");
            }
        }
        previousHit=hit;previousMiss=miss;previousTip=tip;previousNoTip=noTip;
    }
    private void recoverInventory(RobotCoordinator.Input command) {
        boolean pollen=gamepad2.dpad_up,nectar=gamepad2.dpad_down,undo=gamepad2.x,clear=gamepad2.y;
        boolean commit=gamepad2.back&&gamepad2.start;
        if(!command.enabled&&!command.shoot&&!command.intake) {
            io.stop(); // Operator reconciliation never runs with mechanism outputs enabled.
            if(clear&&!previousClear)recoveryDraft.clear();
            if(undo&&!previousUndo&&!recoveryDraft.isEmpty())recoveryDraft.remove(recoveryDraft.size()-1);
            if(pollen&&!previousPollen&&recoveryDraft.size()<4)recoveryDraft.add(BallType.POLLEN);
            if(nectar&&!previousNectar&&recoveryDraft.size()<4)recoveryDraft.add(BallType.NECTAR);
            if(commit&&!previousCommit)robot.inventory.reconcile(recoveryDraft);
        }
        previousPollen=pollen;previousNectar=nectar;previousUndo=undo;previousClear=clear;previousCommit=commit;
    }
    protected void display() {
        if(robot==null) {telemetry.addData("MOTION DISABLED",configurationFault);telemetry.addLine("Configure RobotHardwareConfig with measured hardware/recipes and bench acceptance.");return;}
        telemetry.addData("Scoring",robot.scoring.state());telemetry.addData("Inventory",robot.inventory.load());
        telemetry.addData("Subsystem health",robot.snapshot==null?"AWAITING SAMPLE":robot.snapshot.health);
        telemetry.addData("Drive recovery",robot.driveRecovery()?"ROBOT CENTRIC / ALL MECHANISMS INHIBITED":"Disabled + neutral BACK+Y after a fault");
        telemetry.addData("Outcome shot ID",robot.journal==null?0:robot.journal.lastConfirmedShotId());
        telemetry.addData("Inventory confirmed",robot.inventory.isConfirmed());telemetry.addData("Recovery draft",recoveryDraft);
        telemetry.addData("Balls in flight",robot.scoring.inFlight().size());telemetry.addData("Shot confidence",robot.shotConfidence);
        if(robot.shotSolution!=null) {
            telemetry.addData("Shot range (in)",robot.shotSolution.equivalentRange);
            telemetry.addData("Predicted miss bound (in)",robot.shotMissBoundInches);
        }
        telemetry.addData("Measured shot tip probability",robot.shotTipProbability);
        telemetry.addData("Shot log drops",robot.journal==null?0:robot.journal.dropped);
        telemetry.addData("Pitch ready",robot.scoring.pitchReady);telemetry.addData("Compression ready",robot.scoring.compressionReady);
        telemetry.addData("RPM ready",robot.scoring.rpmReady);
        telemetry.addData("Inhibit",robot.reason);telemetry.addData("Pickup strategy",robot.plan==null?"none":robot.plan.getDecision());
        telemetry.addData("Pickup assist",settings.pickupMotion==null?"awaits measured motion settings":pickupReason);
        telemetry.addLine("Hold RB enable; stick right clockwise; stick up away from alliance wall; A ordinary shot (G2 RB + hold A for validated burst); G2 LB pre-spin; LT intake; B cancel; LB unwind assist; neutral BACK+START reset");
        telemetry.addLine("Hold X with neutral sticks: one confirmed pickup or scoring-position move; release/repress to replan.");
        telemetry.addLine("Disabled recovery G2: UP pollen, DOWN nectar in feed order; X undo; Y clear; BACK+START confirm inspected queue.");
        telemetry.addLine("Observer outcome for displayed last confirmed shot: G2 A hit / B miss; left stick click tip / right click no tip.");
    }
    @Override public void stop(){if(io!=null)io.close();}
}
