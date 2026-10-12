package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.ScoringController;

/** Explicit measured fixed route; faults switch to park only while localization/path remain safe. */
@Autonomous(name="BIOBUZZ Fixed Route",group="Competition")
public final class BioBuzzAutonomous extends BioBuzzTeleOp {
    private FixedRouteAuto auto;
    private int previousCount;
    private double startSec;
    private boolean cancelled;
    private String autoReason="AWAITING CONFIGURATION";
    @Override public void init() {
        super.init();
        if(robot!=null)try {auto=settings.autonomous();}
        catch(RuntimeException failure){configurationFault=failure.getMessage();io.close();io=null;robot=null;}
    }
    @Override public void start() {
        super.start();startSec=seconds(System.nanoTime());
        if(io!=null)try {io.setStartingPose(settings.startX,settings.startY,settings.startHeading);previousCount=robot.inventory.controlledCount();}
        catch(RuntimeException failure){configurationFault=failure.getMessage();io.close();io=null;robot=null;}
    }
    @Override protected RobotCoordinator.Input input() {
        RobotCoordinator.Input input=new RobotCoordinator.Input();
        cancelled |= gamepad1.b;
        if(cancelled){autoReason="DRIVER CANCELLED";input.cancel=true;return input;}
        if(auto==null||robot.snapshot==null)return input;
        double now=seconds(System.nanoTime());
        boolean pickup=robot.inventory.controlledCount()>previousCount;
        previousCount=robot.inventory.controlledCount();
        FixedRouteAuto.Command command=auto.update(robot.snapshot.robot,now,robot.scoring.completedRequestId(),pickup,robot.scoring.state()==ScoringController.State.FAULT);
        autoReason=command.action+": "+command.reason;
        input.intake=auto.intakeRequested();
        input.enabled=command.action!=FixedRouteAuto.Action.DONE&&command.action!=FixedRouteAuto.Action.FAULT;
        input.fieldX=command.xPower;input.fieldY=command.yPower;input.turn=command.turnPower;
        input.shotRequestId=auto.shotRequestId();
        input.mechanismsInhibited=auto.departing();
        input.cancelShot=command.action==FixedRouteAuto.Action.PARK;
        input.cancel=now-startSec>=30;
        if(now-startSec>=30)input.enabled=false;
        return input;
    }
    @Override protected void display() {super.display();telemetry.addData("Autonomous",autoReason);}
}
