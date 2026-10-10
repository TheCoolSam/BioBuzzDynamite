import java.util.*;
import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.turret.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.vision.pieces.*;

/** Diagnostic reproductions against the unchanged production code. */
public final class AuditProbe {
    static RobotState robot(double t, double x, double y) {
        return new RobotState(t,x,y,0,0,0,0,0,0,0,true,true);
    }
    static TurretState aim(double degrees, double turretDegrees) {
        TurretState s = new TurretState();
        s.targetX = 100*Math.cos(Math.toRadians(degrees));
        s.targetY = 100*Math.sin(Math.toRadians(degrees));
        s.turretAngle = Math.toRadians(turretDegrees);
        s.dt = .02;
        return s;
    }
    public static void main(String[] args) {
        TurretController controller = new TurretController();
        TurretCommand a = controller.calculate(aim(179,150));
        TurretCommand b = controller.calculate(aim(181,150));
        System.out.printf(Locale.US,"Rear crossing: desired %.1f -> %.1f deg; power %.2f -> %.2f; unwind %.3f -> %.3f rad/s%n",
            Math.toDegrees(a.desiredTurretAngle),Math.toDegrees(b.desiredTurretAngle),a.turretMotorPower,b.turretMotorPower,a.requestedChassisOmega,b.requestedChassisOmega);
        controller.reset();
        TurretState s = aim(150,150);
        TurretCommand c = null;
        for (int i=0;i<10000;i++) {
            c=controller.calculate(s);
            s.robotHeading += c.requestedChassisOmega*s.dt;
            s.robotAngularVelocity=c.requestedChassisOmega;
            s.turretAngle=c.desiredTurretAngle;
        }
        System.out.printf(Locale.US,"Unwind after 200 simulated seconds: desired %.6f deg; active %s; omega %.9f rad/s; exit threshold %.1f deg%n",
            Math.toDegrees(c.desiredTurretAngle),c.unwindActive,c.requestedChassisOmega,Math.toDegrees(TurretConstants.UNWIND_EXIT_RAD));
        PickupPlanner planner = new PickupPlanner(new MapTipModel(),new EuclideanTravelTimeModel(),new FixedShotSetupModel(.8));
        PickupPlan empty=planner.plan(robot(0,0,0),BallLoad.empty(),Collections.<TrackedPiece>emptyList(),0,null);
        System.out.println("Empty inventory, no known tip rates: decision="+empty.getDecision()+", utility="+empty.getUtility());
        planner = new PickupPlanner(new MapTipModel().set(1,0,1),new EuclideanTravelTimeModel(),new FixedShotSetupModel(.8));
        TrackedPiece edge = new TrackedPiece(1,BallType.POLLEN,PieceOwnership.NEUTRAL,143,50,1,10,true);
        PickupPlan wall=planner.plan(robot(10,120,50),BallLoad.empty(),Arrays.asList(edge),10,null);
        System.out.printf(Locale.US,"Wall pickup: decision %s; capture x %.1f in (beyond nominal 144-in field)%n",wall.getDecision(),wall.getEndpointX());
        PickupPlan stale=planner.plan(robot(0,120,50),BallLoad.empty(),Arrays.asList(edge),10,null);
        System.out.println("10-second-old robot snapshot: decision="+stale.getDecision()+", valid="+stale.isValid());
        RobotStateHistory history=new RobotStateHistory(); history.add(robot(0,0,0));
        PieceTracker tracker=new PieceTracker(HomographyFloorProjection.fromCoefficients(new double[]{1,0,0,0,1,0,0,0,1}),PieceTrackerConstants.defaults());
        PieceObservation observation=PieceObservation.of(1,20,10,4,4,0);
        PieceTrackingResult once=tracker.update(Arrays.asList(observation),history,0);
        PieceTrackingResult repeated=once;
        for(int i=1;i<5;i++) repeated=tracker.update(Arrays.asList(observation),history,0);
        System.out.printf(Locale.US,"Same observation timestamp processed five times: confidence %.2f -> %.2f%n",once.getPieces().get(0).getConfidence(),repeated.getPieces().get(0).getConfidence());
        PieceTrackingResult coast=tracker.update(Collections.<PieceObservation>emptyList(),history,.1);
        System.out.println("No detection after presumed collection: old track still published="+!coast.getPieces().isEmpty());
    }
}
