package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.PedroManualDrive;
import org.firstinspires.ftc.teamcode.planning.pickup.*;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.turret.*;

import java.lang.reflect.Proxy;
import java.util.*;

/** Permanent no-hardware regressions; drive checks execute the real Pedro 3.0.1 follower. */
public final class SafetyRegressionScenarios {
    private static int checks;
    private static int failures;

    public static void main(String[] args) {
        run("D1 stale drive", SafetyRegressionScenarios::driveFailStop);
        run("D2 rear branch", SafetyRegressionScenarios::rearBranch);
        run("D3 unwind completion", SafetyRegressionScenarios::unwindCompletion);
        run("D4 empty shot", SafetyRegressionScenarios::plannerDecisions);
        run("D5 field capture", SafetyRegressionScenarios::fieldCapture);
        if (failures != 0) throw new AssertionError(failures + " safety groups failed");
        System.out.println("SAFETY REGRESSION CHECKS PASSED (" + checks + ")");
    }

    private static void run(String name, Runnable test) {
        try { test.run(); System.out.println(name + " PASS"); }
        catch (AssertionError failure) {
            failures++;
            System.out.println(name + " FAIL: " + failure.getMessage());
        }
    }

    private static void driveFailStop() {
        TestLocalizer localizer = new TestLocalizer();
        TestDrive drive = new TestDrive();
        Algorithm algorithm = (Algorithm) Proxy.newProxyInstance(Algorithm.class.getClassLoader(),
                new Class<?>[] {Algorithm.class}, (proxy, method, values) -> null);
        Follower follower = new Follower(localizer, drive, algorithm);
        check(PedroManualDrive.request(follower, .5, 0, 0), "forward accepted");
        follower.update(.02);
        near(drive.last.forward(), .5, "initial forward output");
        localizer.setPose(new Pose(0, 0, Double.NaN));
        check(!PedroManualDrive.request(follower, 0, 0, 0), "invalid heading rejected");
        follower.update(.02);
        zero(drive.last);
        localizer.setPose(new Pose(0, 0, 0));
        check(PedroManualDrive.request(follower, .3, -.2, .4), "translation/rotation accepted");
        follower.update(.02);
        check(!PedroManualDrive.request(follower, Double.NaN, 0, 0), "invalid input rejected");
        follower.update(.02);
        zero(drive.last);
        follower.update(.02);
        zero(drive.last);
        check(PedroManualDrive.request(follower, -.4, .1, -.2), "recovered command accepted");
        follower.update(.02);
        near(drive.last.forward(), -.4, "recovered forward");
        near(drive.last.strafe(), .1, "recovered strafe");
        near(drive.last.turn(), -.2, "recovered turn");
        for (double bad : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            for (int component = 0; component < 3; component++) {
                check(PedroManualDrive.request(follower, .3, -.2, .4), "fresh demand before rejection");
                follower.update(.02);
                check(!PedroManualDrive.request(follower, component == 0 ? bad : 0,
                        component == 1 ? bad : 0, component == 2 ? bad : 0), "bad component rejected");
                follower.update(.02);
                zero(drive.last);
            }
        }
        for (Pose badPose : new Pose[] {null, new Pose(Double.NaN, 0, 0), new Pose(0, Double.NaN, 0)}) {
            localizer.setPose(new Pose(0, 0, 0));
            PedroManualDrive.request(follower, .3, -.2, .4);
            follower.update(.02);
            localizer.setPose(badPose);
            check(!PedroManualDrive.request(follower, 0, 0, 0), "bad/missing pose rejected");
            follower.update(.02);
            zero(drive.last);
        }
        check(localizer.resets == 0, "no localization reset");
        check(!PedroManualDrive.request(null, 0, 0, 0), "missing follower rejected");
    }

    private static void rearBranch() {
        for (int side : new int[] {1, -1}) {
            TurretController controller = new TurretController();
            for (double bearing : new double[] {175, 178, 179, 180, 181, 182, 185,
                    179.5, -179.5, 179.7, -179.7}) {
                TurretCommand command = controller.calculate(aim(bearing * side, 150 * side));
                near(command.desiredTurretAngle, side > 0
                        ? TurretConstants.OPERATING_MAX_RAD : TurretConstants.OPERATING_MIN_RAD,
                        "rear boundary keeps measured side at " + bearing);
                check(command.requestedChassisOmega * side > 0, "unwind keeps direction");
                legal(command);
            }
            TurretState recovered = aim(179 * side, 100 * side);
            recovered.robotHeading = Math.toRadians(79 * side);
            TurretCommand ordinary = controller.calculate(recovered);
            near(ordinary.desiredTurretAngle, Math.toRadians(100 * side), "ordinary tracking resumes");
            check(!ordinary.unwindActive && ordinary.targetReachable, "branch releases inside exit region");
            legal(ordinary);
            // A latched rear branch does not switch at the opposite operating
            // boundary even after the bearing itself becomes nominally legal.
            controller.reset();
            controller.calculate(aim(179 * side, 150 * side));
            TurretCommand opposite = controller.calculate(aim(-159 * side, 150 * side));
            check(opposite.desiredTurretAngle * side > 0, "opposite edge keeps recovery side");
            check(opposite.requestedChassisOmega * side > 0, "opposite edge keeps yaw direction");
            // A legal front-sector move can resume while the measured turret is
            // still settling; the latch releases only after the encoder follows.
            TurretCommand settling = controller.calculate(aim(100 * side, 150 * side));
            near(settling.desiredTurretAngle, Math.toRadians(100 * side), "legal interior aim resumes");
            check(settling.unwindActive, "does not release before measured turret recovers");
            check(!controller.calculate(aim(100 * side, 100 * side)).unwindActive, "measured recovery releases");
            TurretCommand otherSide = controller.calculate(aim(-150 * side, -150 * side));
            check(otherSide.requestedChassisOmega * side < 0, "released branch can select other side");
            controller.reset();
            check(!controller.calculate(aim(0, 0)).unwindActive, "reset clears recovery");
        }
        TurretController example = new TurretController();
        TurretCommand before = example.calculate(aim(179, 150));
        TurretCommand after = example.calculate(aim(181, 150));
        System.out.println("Rear 179 -> 181: desired " + Math.toDegrees(before.desiredTurretAngle)
                + " -> " + Math.toDegrees(after.desiredTurretAngle) + " deg; omega "
                + before.requestedChassisOmega + " -> " + after.requestedChassisOmega);
    }

    private static void unwindCompletion() {
        for (int side : new int[] {1, -1}) {
            TurretController controller = new TurretController();
            TurretState state = aim(150 * side, 150 * side);
            TurretCommand command = null;
            int loops = 0;
            for (; loops < 10000; loops++) {
                command = controller.calculate(state);
                legal(command);
                if (!command.unwindActive) break;
                check(command.requestedChassisOmega * side > 0, "latched unwind never stalls or reverses");
                state.robotHeading += command.requestedChassisOmega * state.dt;
                state.robotAngularVelocity = command.requestedChassisOmega;
                state.turretAngle = command.desiredTurretAngle;
            }
            check(command != null && !command.unwindActive, "unwind must finish; angle="
                    + Math.toDegrees(command.desiredTurretAngle));
            check(Math.abs(command.desiredTurretAngle) <= TurretConstants.UNWIND_EXIT_RAD,
                    "release uses exit region");
            check(Math.abs(state.turretAngle) <= TurretConstants.UNWIND_EXIT_RAD,
                    "measured turret inside exit region");
            System.out.println("Unwind side " + side + " releases in " + loops * state.dt + " s");
            controller.reset();
            double previousOmega = 0;
            for (double noisyBearing : new double[] {179.5, -179.5, 179.7, -179.7, 179.5, -179.5}) {
                TurretCommand noisy = controller.calculate(aim(noisyBearing, 150 * side));
                check(noisy.unwindActive && noisy.requestedChassisOmega * side > 0, "rear noise keeps recovery yaw");
                check(Math.abs(noisy.requestedChassisOmega) - Math.abs(previousOmega)
                        <= TurretConstants.UNWIND_OMEGA_RAMP_RAD_PER_SEC2 * .02 + 1e-9, "entry yaw ramps");
                previousOmega = noisy.requestedChassisOmega;
                legal(noisy);
            }
        }
    }

    private static void plannerDecisions() {
        PickupPlanner planner = planner(new MapTipModel());
        PickupPlan empty = planner.plan(robot(72, 72), BallLoad.empty(), Collections.emptyList(), 0, null);
        check(empty.getDecision().name().equals("WAIT"), "empty load must WAIT, got " + empty.getDecision());
        TrackedPiece illegal = new TrackedPiece(1, BallType.POLLEN, PieceOwnership.NEUTRAL,
                80, 72, 1, 0, false);
        check(planner.plan(robot(72, 72), BallLoad.empty(), Arrays.asList(illegal), 0, null)
                .getDecision().name().equals("WAIT"), "noncollectable pieces cannot authorize a shot");
        check(planner.plan(robot(72, 72), new BallLoad(1, 0), Collections.emptyList(), 0, null)
                .getDecision().name().equals("WAIT"), "missing model entry cannot authorize a shot");
        check(planner.plan(robot(72, 72), BallLoad.empty(), Arrays.asList(ball(1, 80, 72)), 0, null)
                .getDecision() == PickupPlan.Decision.WAIT, "zero-valued pickup does not win ambiguously");
        PickupPlanner zeroMeasured = planner(new MapTipModel().set(1, 0, 0));
        check(zeroMeasured.plan(robot(72, 72), new BallLoad(1, 0), Collections.emptyList(), 0, null)
                .getDecision() == PickupPlan.Decision.WAIT, "explicit zero is not a meaningful shot");
        PickupPlanner positiveEmpty = planner(new MapTipModel().set(0, 0, 1));
        check(positiveEmpty.plan(robot(72, 72), BallLoad.empty(), Collections.emptyList(), 0, null)
                .getDecision() == PickupPlan.Decision.WAIT, "even positive empty-load model cannot authorize shot");
        planner = planner(new MapTipModel().set(1, 0, .05).set(2, 0, .95));
        check(planner.plan(robot(72, 72), new BallLoad(1, 0), Collections.emptyList(), 0, null)
                .getDecision() == PickupPlan.Decision.SHOOT_NOW, "positive onboard shot remains possible");
        check(planner.plan(robot(72, 72), new BallLoad(1, 0), Arrays.asList(ball(1, 80, 72)), 0, null)
                .getDecision() == PickupPlan.Decision.PICKUP, "useful pickup beats weak shot");
        check(planner.plan(RobotState.invalid(0), BallLoad.empty(), Collections.emptyList(), 0, null)
                .getDecision() == PickupPlan.Decision.INVALID, "invalid pose remains INVALID");
    }

    private static void fieldCapture() {
        PickupPlanner planner = planner(new MapTipModel().set(1, 0, 1));
        PickupPlan plan = planner.plan(robot(120, 50), BallLoad.empty(), Arrays.asList(ball(1, 143, 50)), 0, null);
        check(plan.getDecision() == PickupPlan.Decision.PICKUP, "wall piece has alternate approach");
        check(plan.getEndpointX() >= 0 && plan.getEndpointX() <= 144,
                "capture x must stay in field, got " + plan.getEndpointX());
        // All dimensions/clearances below are TEST fixtures, not robot CAD data.
        FieldBoundsCaptureFeasibility bounds = new FieldBoundsCaptureFeasibility(144, 144, 4, 5);
        planner = new PickupPlanner(new MapTipModel().set(1, 0, 1), new EuclideanTravelTimeModel(),
                new FixedShotSetupModel(.8), bounds);
        double[][] walls = {{143, 72, 120, 72}, {1, 72, 24, 72},
                {72, 143, 72, 120}, {72, 1, 72, 24}};
        for (double[] wall : walls) {
            TrackedPiece piece = ball(1, wall[0], wall[1]);
            RobotState start = robot(wall[2], wall[3]);
            PickupTarget naive = CaptureGeometry.through(piece, wall[2], wall[3], 0, 6);
            check(!bounds.isFeasible(wall[2], wall[3], 0, naive), "naive wall approach infeasible");
            PickupPlan alternate = planner.plan(start, BallLoad.empty(), Arrays.asList(piece), 0, null);
            check(alternate.getDecision() == PickupPlan.Decision.PICKUP, "alternate wall approach found");
            check(alternate.getTargets().size() == 1, "one wall capture");
            check(bounds.isFeasible(wall[2], wall[3], 0, alternate.getTargets().get(0)), "wall capture legal");
            check(alternate.getEndpointX() >= 4 && alternate.getEndpointX() <= 140
                    && alternate.getEndpointY() >= 5 && alternate.getEndpointY() <= 139, "clearances enforced");
        }
        FieldBoundsCaptureFeasibility largerBody = new FieldBoundsCaptureFeasibility(144, 144, 8, 8);
        planner = new PickupPlanner(new MapTipModel().set(1, 0, 1), new EuclideanTravelTimeModel(),
                new FixedShotSetupModel(.8), largerBody);
        for (double[] wall : walls) {
            PickupPlan unreachable = planner.plan(robot(wall[2], wall[3]), BallLoad.empty(),
                    Arrays.asList(ball(1, wall[0], wall[1])), 0, null);
            check(unreachable.getDecision() == PickupPlan.Decision.WAIT && unreachable.getTargets().isEmpty(),
                    "genuinely unreachable wall piece excluded");
        }
        PickupTarget naiveMidfield = CaptureGeometry.through(ball(2, 80, 72), 72, 72, 0, 6);
        PickupPlan midfield = planner.plan(robot(72, 72), BallLoad.empty(), Arrays.asList(ball(2, 80, 72)), 0, null);
        check(midfield.getDecision() == PickupPlan.Decision.PICKUP, "ordinary midfield pickup");
        near(midfield.getEndpointX(), naiveMidfield.getCaptureX(), "midfield x unchanged");
        near(midfield.getEndpointY(), naiveMidfield.getCaptureY(), "midfield y unchanged");
        near(midfield.getEndpointHeadingRad(), naiveMidfield.getApproachHeadingRad(), "midfield heading unchanged");
        check(planner.plan(robot(0, 72), BallLoad.empty(), Arrays.asList(ball(2, 80, 72)), 0, null)
                .getDecision() == PickupPlan.Decision.WAIT, "illegal start center excluded");

        final boolean[] allow = {true};
        final int[] feasibilityCalls = {0};
        final int[] travelCalls = {0};
        CaptureFeasibilityModel injected = (x, y, heading, target) -> {
            feasibilityCalls[0]++;
            return allow[0] && bounds.isFeasible(x, y, heading, target);
        };
        TravelTimeModel cost = (x, y, heading, target) -> { travelCalls[0]++; return 1; };
        planner = new PickupPlanner(new MapTipModel().set(1, 0, 1), cost, new FixedShotSetupModel(.8), injected);
        PickupPlan committed = planner.plan(robot(72, 72), BallLoad.empty(), Arrays.asList(ball(2, 80, 72)), 0, null);
        check(committed.getDecision() == PickupPlan.Decision.PICKUP, "initial feasible commitment");
        allow[0] = false;
        feasibilityCalls[0] = travelCalls[0] = 0;
        PickupPlan rejected = planner.plan(robot(72, 72), BallLoad.empty(), Arrays.asList(ball(2, 80, 72)), 0, committed);
        check(rejected.getDecision() == PickupPlan.Decision.WAIT && rejected.getTargets().isEmpty(),
                "injected rejection also invalidates commitment");
        check(travelCalls[0] == 0, "impossible approaches never enter travel scoring");
        check(feasibilityCalls[0] == 18, "bounded nine trials each for search and commitment");

        // Re-check each leg from the preceding capture, not just the initial robot.
        final double[] secondStartX = {Double.NaN};
        CaptureFeasibilityModel ordered = (x, y, heading, target) -> {
            if (target.getPiece().getId() == 2) secondStartX[0] = x;
            return bounds.isFeasible(x, y, heading, target)
                    && (target.getPiece().getId() == 1 || x == 86);
        };
        planner = new PickupPlanner(new MapTipModel().set(2, 0, 1), new EuclideanTravelTimeModel(),
                new FixedShotSetupModel(.8), ordered);
        PickupPlan route = planner.plan(robot(72, 72), BallLoad.empty(),
                Arrays.asList(ball(1, 80, 72), ball(2, 100, 72)), 0, null);
        check(route.getTargets().size() == 2, "route-dependent feasibility uses successive endpoints");
        check(route.getTargets().get(0).getPiece().getId() == 1, "feasible order retained");
        double x = 72, y = 72, heading = 0;
        for (PickupTarget target : route.getTargets()) {
            check(ordered.isFeasible(x, y, heading, target), "no returned target was rejected");
            x = target.getCaptureX(); y = target.getCaptureY(); heading = target.getApproachHeadingRad();
        }
        near(secondStartX[0], 86, "second capture starts at first endpoint");
    }

    private static PickupPlanner planner(MapTipModel tips) {
        return new PickupPlanner(tips, new EuclideanTravelTimeModel(), new FixedShotSetupModel(.8));
    }

    private static TrackedPiece ball(int id, double x, double y) {
        return new TrackedPiece(id, BallType.POLLEN, PieceOwnership.NEUTRAL, x, y, 1, 0, true);
    }

    private static RobotState robot(double x, double y) {
        return new RobotState(0, x, y, 0, 0, 0, 0, 0, 0, 0, true, true);
    }

    private static TurretState aim(double bearingDegrees, double turretDegrees) {
        TurretState state = new TurretState();
        state.targetX = 100 * Math.cos(Math.toRadians(bearingDegrees));
        state.targetY = 100 * Math.sin(Math.toRadians(bearingDegrees));
        state.turretAngle = Math.toRadians(turretDegrees);
        state.dt = .02;
        return state;
    }

    private static void legal(TurretCommand command) {
        check(command.desiredTurretAngle >= TurretConstants.OPERATING_MIN_RAD
                && command.desiredTurretAngle <= TurretConstants.OPERATING_MAX_RAD, "legal operating angle");
        check(Math.abs(command.requestedChassisOmega) <= TurretConstants.MAX_REQUESTED_CHASSIS_OMEGA,
                "bounded chassis request");
    }

    private static void zero(DrivePowers powers) {
        near(powers.forward(), 0, "forward cleared");
        near(powers.strafe(), 0, "strafe cleared");
        near(powers.turn(), 0, "turn cleared");
    }

    private static void near(double actual, double expected, String label) {
        check(Math.abs(actual - expected) <= 1e-9, label + ": expected " + expected + ", got " + actual);
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) throw new AssertionError(label);
    }

    private static final class TestLocalizer implements Localizer {
        MotionState value = MotionState.zero();
        Pose reportedPose = new Pose(0, 0, 0);
        int resets;
        public void setPose(Pose pose) {
            reportedPose = pose;
            if (pose != null) value = value.withPose(pose);
        }
        public Pose pose() { return reportedPose; }
        public MotionState state() { return value; }
        public void update() { }
        public void reset() { resets++; value = MotionState.zero(); }
    }

    private static final class TestDrive implements Drivetrain {
        DrivePowers last = DrivePowers.zero();
        public void drive(DrivePowers powers, boolean manual) { last = powers; }
        public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
        public void stop() { last = DrivePowers.zero(); }
        public void stop(boolean brake) { stop(); }
        public Map<String, Object> debug() { return Collections.emptyMap(); }
        public double interpolateVelocity(double a, double b, double c) { return 1; }
    }
}
