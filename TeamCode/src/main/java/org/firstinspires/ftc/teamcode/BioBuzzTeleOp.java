package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import java.util.*;

/** Integrated stationary-scoring TeleOp. Remains powerless until measured settings exist. */
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
    @Override public void init() {
        epochNanos=lastNanos=System.nanoTime();telemetry.setMsTransmissionInterval(100);
        settings=RobotHardwareConfig.measured();
        if(settings!=null)try {
            settings.validate();io=new RevRobotIO(hardwareMap,settings,epochNanos);
            PieceInventory inventory=new PieceInventory(4);inventory.reconcile(settings.confirmedPreload);
            ScoringController scoring=new ScoringController(inventory,settings.recipes,settings.scoringTiming,settings.feedPower,settings.reversePower);
            robot=new RobotCoordinator(io,inventory,scoring,new PieceTracker(settings.floor,settings.pieces),
                    new PickupPlanner(settings.tipModel,settings.travel,settings.shots,settings.footprint),
                    new TurretController(settings.turretProfile),new DriveAssist(settings.yawAtFullPower,.3,.08),
                    new HiveTracker(settings.hiveSettleSec,settings.hiveMaxAgeSec),settings.aimTolerance,settings.maxMotion,settings.maxTurretRate);
            robot.setIntakePower(settings.intakePower);
            robot.setTurretPivot(settings.camera.pivotForward(),settings.camera.pivotLeft());
            robot.setScoringPoses(settings.shots,settings.scoringPositionTolerance,settings.scoringHeadingTolerance);
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
        input.unwindAssist=gamepad1.left_bumper;
        input.fieldX=deadband(gamepad1.left_stick_x);input.fieldY=deadband(-gamepad1.left_stick_y);
        input.turn=deadband(gamepad1.right_stick_x);
        boolean hold=gamepad1.x;
        boolean neutral=input.fieldX==0&&input.fieldY==0&&input.turn==0;
        input.resetFault &= neutral;
        if(!hold||!input.enabled||input.cancel||!neutral||input.shoot||robot==null||!robot.inventory.isConfirmed()
                ||robot.scoring.state()!=ScoringController.State.IDLE)pickup=null;
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
    private static double deadband(double v){return Math.abs(v)<.08?0:v;}
    @Override public void loop() {
        long now=System.nanoTime();double dt=(now-lastNanos)*1e-9;lastNanos=now;
        if(robot!=null) {
            RobotCoordinator.Input command=input();
            recoverInventory(command);
            robot.cycle(command,seconds(now),dt);
        }
        metrics.record((System.nanoTime()-now)*1e-9);
        if(robot!=null&&seconds(now)-lastLog>=.5) {
            double[] timing=metrics.summary();
            RobotLog.ii("BioBuzz","t=%.3f scoring=%s inventory=%s inhibit=%s loopP50=%.4f loopP95=%.4f loopP99=%.4f max=%.4f",
                    seconds(now),robot.scoring.state(),robot.inventory.load(),robot.reason,timing[0],timing[1],timing[2],timing[3]);lastLog=seconds(now);
        }
        display();
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
        telemetry.addData("Inventory confirmed",robot.inventory.isConfirmed());telemetry.addData("Recovery draft",recoveryDraft);
        telemetry.addData("Inhibit",robot.reason);telemetry.addData("Pickup strategy",robot.plan==null?"none":robot.plan.getDecision());
        telemetry.addData("Pickup assist",settings.pickupMotion==null?"awaits measured motion settings":pickupReason);
        telemetry.addLine("Hold RB enable; A shot; LT intake; B cancel; LB unwind assist; neutral BACK+START reset");
        telemetry.addLine("Hold X with neutral sticks: one confirmed pickup or scoring-position move; release/repress to replan.");
        telemetry.addLine("Disabled recovery G2: UP pollen, DOWN nectar in feed order; X undo; Y clear; BACK+START confirm inspected queue.");
    }
    @Override public void stop(){if(io!=null)io.close();}
}
