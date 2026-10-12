import java.util.Arrays;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.robot.*;
import org.firstinspires.ftc.teamcode.scoring.*;
import org.firstinspires.ftc.teamcode.state.RobotState;

/** Historical reproductions of the pre-fix one-pulse adapter and circle-only departure.
 * Use CompetitionReadinessScenarios in the desktop runner to verify the new contracts.
 * All numbers are synthetic, not robot calibration. */
public final class WorldsReadinessProbe {
    private static RobotState pose(double time, double x, double y) {
        return new RobotState(time,x,y,0,0,0,0,0,0,0,true,true,time,true,0);
    }
    private static ScoringController.Input input(double time) {
        ScoringController.Input in=new ScoringController.Input();
        in.nowSec=in.acquiredSec=in.hiveObservedSec=in.targetObservedSec=time;
        in.healthy=in.enabled=in.staged=in.aimReady=in.targetFresh=in.stationary=in.hiveStable=true;
        in.rpm=1000;in.rangeInches=80;return in;
    }
    private static ScoringController scorer(boolean requireRange) {
        PieceInventory inventory=new PieceInventory(4);
        inventory.reconcile(Arrays.asList(BallType.POLLEN,BallType.POLLEN));
        ShotRecipe recipe=new ShotRecipe(BallType.POLLEN,1000,.4,.6,.1,50,.1);
        ShotSelector selector=(type,range)->requireRange&&!Double.isFinite(range)?null:
                new ShotSelector.Selection(recipe,Double.NaN,Double.NaN);
        return new ScoringController(inventory,selector,
                new ScoringController.Timing(.5,.1,.2,.1,.5,.2,0),.3,.2,null);
    }
    private static FixedRouteAuto route(SweptCaptureFeasibility footprint,double x,double y,boolean shoot) {
        return new FixedRouteAuto(new FixedRouteAuto.Waypoint[]{
                new FixedRouteAuto.Waypoint(x,y,0,.5,shoot)},
                new FixedRouteAuto.Waypoint(50,50,0,5,false),footprint,
                30,25,1,.05,.05,1,.3);
    }
    private static void missingSelectionLosesAutoRequest() {
        ScoringController scoring=scorer(true);
        FixedRouteAuto auto=route(new SweptCaptureFeasibility(144,144,5,null),40,40,true);
        boolean previousShoot=false;
        for(int step=0;step<=6;step++) {
            double time=step*.1;
            FixedRouteAuto.Command command=auto.update(pose(time,40,40),time,false,false);
            boolean shoot=command.action==FixedRouteAuto.Action.SHOOT;
            ScoringController.Input in=input(time);
            // Exactly the one-pulse translation in BioBuzzAutonomous.input().
            in.start=shoot&&!previousShoot;previousShoot=shoot;
            if(step==0)in.rangeInches=Double.NaN;
            scoring.update(in);
            if(scoring.state()!=ScoringController.State.IDLE)
                throw new AssertionError("Expected dropped request to leave scoring idle");
            if(step==6&&command.action!=FixedRouteAuto.Action.PARK)
                throw new AssertionError("Expected waypoint timeout to park");
        }
        System.out.println("Late target: scoring remains IDLE; autonomous times out and parks without firing.");
    }
    private static void busyScorerLosesNextRequest() {
        ScoringController scoring=scorer(false);
        ScoringController.Input in=input(0);in.start=true;scoring.update(in);
        scoring.update(input(.11));in=input(.15);in.exit=true;scoring.update(in);
        if(scoring.state()!=ScoringController.State.WAIT_FOR_HIVE)throw new AssertionError("Setup");
        // Next waypoint sends its only start pulse while the previous shot is reacquiring.
        in=input(.20);in.start=true;scoring.update(in);
        scoring.update(input(.30));scoring.update(input(.35));
        if(scoring.state()!=ScoringController.State.IDLE)throw new AssertionError("Expected lost request");
        System.out.println("Next shot while WAIT_FOR_HIVE: start pulse is discarded; scorer later idles without firing.");
    }
    private static void legalWallStartRejected() {
        // Illustrative 18-inch square, center 9 inches from wall, with a correctly enclosing circle.
        double radius=Math.hypot(9,9);
        SweptCaptureFeasibility footprint=new SweptCaptureFeasibility(144,144,radius,null);
        FixedRouteAuto auto=route(footprint,24,40,false);
        FixedRouteAuto.Command command=auto.update(pose(0,9,40),0,false,false);
        if(command.action!=FixedRouteAuto.Action.FAULT)throw new AssertionError("Expected circle rejection");
        System.out.println("Legal wall-contact rectangular start: "+command.action+" / "+command.reason);
    }
    public static void main(String[] args) {
        missingSelectionLosesAutoRequest();busyScorerLosesNextRequest();legalWallStartRejected();
    }
}
