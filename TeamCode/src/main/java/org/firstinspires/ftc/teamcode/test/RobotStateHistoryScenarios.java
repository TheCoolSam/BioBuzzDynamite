package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateHistory;

import java.util.Optional;

/**
 * Desktop checks for timestamped pose history. Not an OpMode.
 */
public final class RobotStateHistoryScenarios {

    private static int checks = 0;

    private RobotStateHistoryScenarios() {
    }

    public static void main(String[] args) {
        testExactLookup();
        testLinearTranslation();
        testHeadingWrap();
        testNoHistory();
        testInvalidBracket();
        testEndpointTolerance();
        testCapacity();
        System.out.println("ROBOT STATE HISTORY CHECKS PASSED (" + checks + ")");
    }

    private static void testExactLookup() {
        RobotStateHistory history = new RobotStateHistory();
        RobotState stored = pose(1.0, 12.0, 4.0, 0.3, 1.0, 2.0, 0.1);
        history.add(stored);
        Optional<RobotState> sample = history.sampleAt(1.0);
        check(sample.isPresent(), "A a stored timestamp is available");
        check(sample.get() == stored, "A the stored snapshot is returned");
    }

    private static void testLinearTranslation() {
        RobotStateHistory history = new RobotStateHistory();
        history.add(pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        history.add(pose(1.0, 40.0, 8.0, 0.0, 40.0, 0.0, 0.0));
        RobotState sample = history.sampleAt(0.25).get();
        near(sample.getFieldX(), 10.0, "B x is one quarter of the way");
        near(sample.getFieldY(), 2.0, "B y is interpolated");
        near(sample.getFieldVx(), 10.0, "B velocity is interpolated");
        check(sample.isPoseValid(), "B the interpolated pose is valid");
        near(sample.getTimestampSec(), 0.25, "B the sample uses the requested time");
    }

    private static void testHeadingWrap() {
        double from = Math.toRadians(179.0);
        double to = Math.toRadians(-179.0);
        RobotStateHistory history = new RobotStateHistory();
        history.add(pose(0.0, 0.0, 0.0, from, 0.0, 0.0, 0.0));
        history.add(pose(1.0, 0.0, 0.0, to, 0.0, 0.0, 0.0));
        RobotState sample = history.sampleAt(0.5).get();
        double heading = sample.getHeadingRad();
        check(Math.abs(Math.abs(heading) - Math.PI) < 1.0e-6, "C the midpoint stays near 180 degrees");
        check(Math.abs(heading) > 1.0, "C the midpoint is not near 0 degrees");
    }

    private static void testNoHistory() {
        RobotStateHistory history = new RobotStateHistory();
        check(!history.sampleAt(0.0).isPresent(), "D an empty history is unavailable");
        history.add(pose(1.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0));
        Optional<RobotState> early = history.sampleAt(0.5);
        check(!early.isPresent(), "D a time before the window is unavailable");
        check(!history.sampleAt(Double.NaN).isPresent(), "D a malformed timestamp is unavailable");
    }

    private static void testInvalidBracket() {
        RobotStateHistory history = new RobotStateHistory();
        history.add(pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        history.add(RobotState.invalid(1.0));
        check(!history.sampleAt(0.5).isPresent(), "E an invalid bracket is not interpolated");
        check(!history.sampleAt(1.0).isPresent(), "E an invalid sample is not a pose");
        Optional<RobotState> start = history.sampleAt(0.0);
        check(start.isPresent() && start.get().isPoseValid(), "E the valid endpoint itself remains available");
        near(start.get().getFieldX(), 0.0, "E the valid endpoint keeps its own x");
    }

    private static void testEndpointTolerance() {
        RobotStateHistory history = new RobotStateHistory();
        RobotState stored = pose(1.0, 3.0, 4.0, 0.2, 0.0, 0.0, 0.0);
        history.add(stored);
        Optional<RobotState> aligned = history.sampleAt(1.0 + RobotStateHistory.DEFAULT_ENDPOINT_TOLERANCE_SEC);
        check(aligned.isPresent() && aligned.get() == stored, "a 2 ms alignment uses the stored snapshot");
        check(!history.sampleAt(1.05).isPresent(), "50 ms outside the window is unavailable");
        check(!history.sampleAt(1.0 - 0.05).isPresent(), "50 ms before the window is unavailable");
    }

    private static void testCapacity() {
        RobotStateHistory history = new RobotStateHistory(1.5, 3, 0.0);
        history.add(pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        history.add(pose(0.2, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        history.add(pose(0.4, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        history.add(pose(0.6, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        check(history.size() == 3, "the sample cap drops the oldest");
        check(!history.sampleAt(0.0).isPresent(), "a dropped sample is gone");
        history.add(pose(3.0, 9.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        check(!history.sampleAt(0.6).isPresent(), "samples older than the duration are dropped");
        check(history.sampleAt(3.0).isPresent(), "the newest sample remains");
    }

    private static RobotState pose(
            double time, double x, double y, double heading, double vx, double vy, double omega) {
        return new RobotState(
                time, x, y, heading, vx, vy, omega, 0.0, 0.0, 0.0, true, true);
    }

    private static void near(double actual, double expected, String label) {
        checks++;
        if (!(Math.abs(actual - expected) <= 1.0e-6)) {
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
