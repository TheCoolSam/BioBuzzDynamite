package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.AngleUtil;
import org.firstinspires.ftc.teamcode.planning.pickup.BallLoad;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.EuclideanTravelTimeModel;
import org.firstinspires.ftc.teamcode.planning.pickup.MapTipModel;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlan;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlanner;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupTarget;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;

import java.util.ArrayList;
import java.util.List;

/**
 * No-hardware view of the pickup planner. Nothing here drives a robot.
 *
 * <p>The tip table below is a demo fixture so the screen has something to
 * show. It is not a measured HIVE model.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: field velocity. Stick up is +Y.</li>
 *   <li>Right stick X: yaw. Stick right is counterclockwise.</li>
 *   <li>A: pretend the first target was collected, then replan.</li>
 *   <li>Dpad down: toggle an invalid pose.</li>
 *   <li>Start: reset pose, load, and pieces.</li>
 * </ul>
 */
@TeleOp(name = "Pickup Planner Test", group = "Test")
public class PickupPlannerTestOpMode extends OpMode {

    private static final double STICK_SPEED_IN_PER_SEC = 48.0;
    private static final double STICK_OMEGA = Math.toRadians(120.0);
    private static final double MAX_LOOP_DT = 0.05;
    private static final double STICK_DEADBAND = 0.08;

    /**
     * Demo fixture only. Two pollen barely tip. Three nectar tip often.
     * One extra pollen on a nectar load is a small bump, not a rule of the game.
     */
    private static final MapTipModel DEMO_TIPS = new MapTipModel()
            .set(2, 0, 0.05)
            .set(4, 0, 0.40)
            .set(0, 3, 0.90)
            .set(1, 3, 0.93);

    private final ElapsedTime timer = new ElapsedTime();
    private final PickupPlanner planner = new PickupPlanner(DEMO_TIPS, new EuclideanTravelTimeModel());
    private final List<TrackedPiece> pieces = new ArrayList<TrackedPiece>();

    private BallLoad load = BallLoad.empty();
    private PickupPlan committed;
    private double x;
    private double y;
    private double heading;
    private double timestampSec;
    private boolean poseInvalid;
    private boolean aWasDown;
    private boolean dpadDownWasDown;
    private boolean startWasDown;

    @Override
    public void init() {
        timer.reset();
        resetField();
        telemetry.addData("Status", "No hardware required");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double now = timer.seconds();
        double dt = now - timestampSec;
        if (dt < 0.0 || dt > MAX_LOOP_DT) {
            dt = MAX_LOOP_DT;
        }
        timestampSec = now;

        if (pressed(gamepad1.start, startWasDown)) {
            resetField();
        }
        startWasDown = gamepad1.start;

        if (pressed(gamepad1.dpad_down, dpadDownWasDown)) {
            poseInvalid = !poseInvalid;
        }
        dpadDownWasDown = gamepad1.dpad_down;

        double vx = deadband(gamepad1.left_stick_x) * STICK_SPEED_IN_PER_SEC;
        double vy = deadband(-gamepad1.left_stick_y) * STICK_SPEED_IN_PER_SEC;
        double omega = deadband(gamepad1.right_stick_x) * STICK_OMEGA;
        x += vx * dt;
        y += vy * dt;
        heading = AngleUtil.wrapRadians(heading + (omega * dt));

        RobotState robot = new RobotState(
                timestampSec, x, y, heading,
                vx, vy, omega,
                0.0, 0.0, 0.0,
                !poseInvalid, true);

        if (pressed(gamepad1.a, aWasDown) && committed != null && !committed.getTargets().isEmpty()) {
            PickupTarget grabbed = committed.getTargets().get(0);
            load = load.plus(grabbed.getPiece().getType());
            removePiece(grabbed.getPiece().getId());
        }
        aWasDown = gamepad1.a;

        // These pieces are scripted, so each loop counts as a fresh sighting.
        markSeen(timestampSec);
        committed = planner.plan(robot, load, pieces, timestampSec, committed);
        publish(robot);
    }

    @Override
    public void stop() {
    }

    private void resetField() {
        x = 0.0;
        y = 0.0;
        heading = 0.0;
        timestampSec = 0.0;
        poseInvalid = false;
        load = BallLoad.empty();
        committed = null;
        pieces.clear();
        pieces.add(piece(1, BallType.POLLEN, 14.0, 2.0));
        pieces.add(piece(2, BallType.POLLEN, 18.0, -3.0));
        pieces.add(piece(3, BallType.NECTAR, 40.0, 6.0));
        pieces.add(piece(4, BallType.NECTAR, 48.0, 0.0));
        pieces.add(piece(5, BallType.NECTAR, 44.0, -8.0));
    }

    private void publish(RobotState robot) {
        telemetry.addData("Pose", robot.isPoseValid() ? "ok" : "INVALID");
        telemetry.addData("Robot in", "%.1f, %.1f", robot.getFieldX(), robot.getFieldY());
        telemetry.addData("Heading deg", "%.1f", AngleUtil.toDegrees(robot.getHeadingRad()));
        telemetry.addData("Onboard", load.toString());
        telemetry.addData("Decision", committed.getDecision());
        telemetry.addData("Utility", "%.2f", committed.getUtility());
        telemetry.addData("Tip p", "%.2f", committed.getTipProbability());
        telemetry.addData("Seconds", "%.2f", committed.getEstimatedSeconds());
        telemetry.addData("Route", routeText(committed));
        telemetry.addLine("LS move | RS yaw | A collect | DD invalid pose | Start reset");
    }

    private static String routeText(PickupPlan plan) {
        if (plan.getTargets().isEmpty()) {
            return plan.getDecision() == PickupPlan.Decision.SHOOT_NOW ? "shoot now" : "no motion";
        }
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < plan.getTargets().size(); i++) {
            TrackedPiece piece = plan.getTargets().get(i).getPiece();
            if (i > 0) {
                text.append(" -> ");
            }
            text.append(piece.getId());
            text.append(piece.getType() == BallType.NECTAR ? "N" : "P");
        }
        return text.toString();
    }

    private void markSeen(double nowSec) {
        for (int i = 0; i < pieces.size(); i++) {
            TrackedPiece piece = pieces.get(i);
            pieces.set(i, new TrackedPiece(
                    piece.getId(),
                    piece.getType(),
                    piece.getOwnership(),
                    piece.getFieldX(),
                    piece.getFieldY(),
                    piece.getConfidence(),
                    nowSec,
                    piece.isCollectable()));
        }
    }

    private void removePiece(int id) {
        for (int i = pieces.size() - 1; i >= 0; i--) {
            if (pieces.get(i).getId() == id) {
                pieces.remove(i);
            }
        }
    }

    private static TrackedPiece piece(int id, BallType type, double fieldX, double fieldY) {
        return new TrackedPiece(
                id,
                type,
                PieceOwnership.NEUTRAL,
                fieldX,
                fieldY,
                1.0,
                0.0,
                true);
    }

    private static boolean pressed(boolean down, boolean wasDown) {
        return down && !wasDown;
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) {
            return 0.0;
        }
        return value;
    }
}
