package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.PedroManualDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroMappedReading;
import org.firstinspires.ftc.teamcode.pedro.PedroPoseAdapter;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupTarget;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateEstimator;

/**
 * No-hardware checks of the Pedro coordinate bridge. Not an OpMode.
 * Does not construct a follower, a localizer, or any tuned coefficient.
 */
public final class PedroMappingScenarios {

    private static int checks = 0;

    private PedroMappingScenarios() {
    }

    public static void main(String[] args) {
        testPositiveX();
        testPositiveY();
        testCounterclockwise();
        testVelocitySample();
        testHeadingWrapDoesNotSpikeAcceleration();
        testInvalidNeverBecomesOrigin();
        testFieldCommandSigns();
        testCapturePoseBecomesPedroPose();
        System.out.println("PEDRO MAPPING CHECKS PASSED (" + checks + ")");
    }

    private static void testPositiveX() {
        PedroMappedReading reading = PedroMappedReading.from(new Pose(12.0, 0.0, 0.0), Velocity.zero());
        near(reading.getX(), 12.0, "A +X stays +X");
        check(reading.isPoseValid(), "A pose is valid");
    }

    private static void testPositiveY() {
        PedroMappedReading reading = PedroMappedReading.from(new Pose(0.0, -7.5, 0.0), Velocity.zero());
        near(reading.getY(), -7.5, "B +Y and -Y keep their sign");
        check(reading.isPoseValid(), "B pose is valid");
    }

    private static void testCounterclockwise() {
        PedroMappedReading reading = PedroMappedReading.from(
                new Pose(0.0, 0.0, 0.4),
                new Velocity(0.0, 0.0, 0.5));
        near(reading.getHeadingRad(), 0.4, "C CCW heading stays positive");
        near(reading.getAngularVelocityRadPerSec(), 0.5, "C CCW omega stays positive");
    }

    private static void testVelocitySample() {
        PedroMappedReading reading = PedroMappedReading.from(
                new Pose(1.0, 2.0, 0.0),
                new Velocity(20.0, -10.0, 0.5));
        near(reading.getFieldVx(), 20.0, "D fieldVx is +20");
        near(reading.getFieldVy(), -10.0, "D fieldVy is -10");
        near(reading.getAngularVelocityRadPerSec(), 0.5, "D omega is +0.5");
        check(reading.isVelocityValid(), "D velocity is valid");
    }

    private static void testHeadingWrapDoesNotSpikeAcceleration() {
        RobotStateEstimator estimator = new RobotStateEstimator();
        Velocity velocity = new Velocity(20.0, -10.0, 0.5);
        estimator.update(
                PedroMappedReading.from(new Pose(1.0, 2.0, Math.PI - 0.02), velocity, 1, true, 0),
                1.0);
        RobotState wrapped = estimator.update(
                PedroMappedReading.from(new Pose(1.0, 2.0, -Math.PI + 0.02), velocity, 1.02, true, 0),
                1.02);
        near(wrapped.getAngularAccelerationRadPerSec2(), 0.0, "E heading wrap does not spike alpha");
        near(wrapped.getFieldAx(), 0.0, "E heading wrap does not spike Ax");
        near(wrapped.getFieldAy(), 0.0, "E heading wrap does not spike Ay");
        check(wrapped.isPoseValid(), "E wrapped pose stays valid");
    }

    private static void testInvalidNeverBecomesOrigin() {
        PedroMappedReading nanPose = PedroMappedReading.from(
                new Pose(Double.NaN, 5.0, 1.0),
                new Velocity(20.0, -10.0, 0.5), 2, true, 0);
        check(!nanPose.isPoseValid(), "F non-finite pose is invalid");
        near(nanPose.getX(), 0.0, "F invalid pose stores 0 instead of NaN");
        check(nanPose.isVelocityValid(), "F a good velocity stays usable");
        near(nanPose.getFieldVx(), 20.0, "F velocity is still +20");

        RobotStateEstimator estimator = new RobotStateEstimator();
        RobotState state = estimator.update(nanPose, 2.0);
        check(!state.isPoseValid(), "F estimator does not treat NaN as the origin");
        near(state.getFieldX(), 0.0, "F estimator pose number is 0");
        check(state.isVelocityValid(), "F estimator keeps the valid velocity");

        PedroMappedReading nanVelocity = PedroMappedReading.from(
                new Pose(4.0, 5.0, 0.2),
                new Velocity(Double.POSITIVE_INFINITY, 0.0, 0.0));
        check(nanVelocity.isPoseValid(), "F bad velocity leaves the pose");
        near(nanVelocity.getX(), 4.0, "F pose x remains 4");
        check(!nanVelocity.isVelocityValid(), "F non-finite velocity is invalid");
        near(nanVelocity.getFieldVx(), 0.0, "F invalid velocity stores 0");

        PedroMappedReading missing = PedroMappedReading.from(null, null);
        check(!missing.isPoseValid(), "F null pose is invalid");
        check(!missing.isVelocityValid(), "F null velocity is invalid");
        near(missing.getX(), 0.0, "F null pose is not NaN");

        PedroMappedReading origin = PedroMappedReading.from(new Pose(0.0, 0.0, 0.0), Velocity.zero());
        check(origin.isPoseValid(), "F a real origin pose stays valid");
    }

    private static void testFieldCommandSigns() {
        DrivePowers facingX = PedroManualDrive.powers(1.0, 0.0, 0.25, 0.0);
        check(facingX != null, "field command at heading 0 exists");
        near(facingX.forward(), 1.0, "heading 0 maps +X to forward");
        near(facingX.strafe(), 0.0, "heading 0 maps +X to no strafe");
        near(facingX.turn(), 0.25, "positive omega stays CCW turn");

        DrivePowers facingXUp = PedroManualDrive.powers(0.0, 1.0, 0.0, 0.0);
        near(facingXUp.forward(), 0.0, "heading 0 maps +Y to no forward");
        near(facingXUp.strafe(), 1.0, "heading 0 maps +Y to strafe-left");

        DrivePowers facingY = PedroManualDrive.powers(1.0, 0.0, 0.0, Math.PI / 2.0);
        near(facingY.forward(), 0.0, "heading 90 maps +X off forward");
        near(facingY.strafe(), -1.0, "heading 90 maps +X to strafe-right");

        check(PedroManualDrive.powers(Double.NaN, 0.0, 0.0, 0.0) == null, "non-finite command is refused");
    }

    private static void testCapturePoseBecomesPedroPose() {
        TrackedPiece piece = new TrackedPiece(
                1, BallType.NECTAR, PieceOwnership.NEUTRAL, 10.0, 20.0, 1.0, 0.0, true);
        PickupTarget target = new PickupTarget(piece, 30.0, -4.0, Math.PI / 2.0);
        Pose pose = PedroPoseAdapter.fromPickupTarget(target);
        check(pose != null, "capture target becomes a pose");
        near(pose.x(), 30.0, "capture x is unchanged");
        near(pose.y(), -4.0, "capture y is unchanged");
        near(pose.heading(), Math.PI / 2.0, "approach heading is unchanged");

        Path path = PedroPoseAdapter.straightToCapture(new Pose(0.0, 0.0, 0.0), target);
        check(path != null, "capture target can start a Pedro path");
        check(PedroPoseAdapter.toPose(1.0, Double.NaN, 0.0) == null, "non-finite capture is not a pose");
        check(PedroPoseAdapter.straightToCapture(new Pose(0.0, 0.0, 0.0), null) == null, "missing target is not a path");
    }

    private static void near(double actual, double expected, String label) {
        checks++;
        if (!(Math.abs(actual - expected) <= 1e-9)) {
            throw new AssertionError(label + " expected " + expected + " but was " + actual);
        }
    }

    private static void check(boolean condition, String label) {
        checks++;
        if (!condition) {
            throw new AssertionError(label);
        }
    }
}
