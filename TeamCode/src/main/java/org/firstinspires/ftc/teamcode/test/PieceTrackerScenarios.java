package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.match.AllianceColor;
import org.firstinspires.ftc.teamcode.planning.pickup.BallLoad;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.EuclideanTravelTimeModel;
import org.firstinspires.ftc.teamcode.planning.pickup.FixedShotSetupModel;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlan;
import org.firstinspires.ftc.teamcode.planning.pickup.PickupPlanner;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
import org.firstinspires.ftc.teamcode.planning.pickup.TipModel;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateHistory;
import org.firstinspires.ftc.teamcode.vision.pieces.FieldPlacement;
import org.firstinspires.ftc.teamcode.vision.pieces.FloorPoint;
import org.firstinspires.ftc.teamcode.vision.pieces.HomographyFloorProjection;
import org.firstinspires.ftc.teamcode.vision.pieces.ObservationAnchor;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceObservation;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTracker;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackerConstants;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackingResult;

import java.util.Arrays;
import java.util.List;

/**
 * No-camera checks for floor projection and piece tracking.
 * Tip numbers used with the planner are fixtures, not HIVE measurements.
 */
public final class PieceTrackerScenarios {

    private static int checks = 0;

    private PieceTrackerScenarios() {
    }

    public static void main(String[] args) {
        testProjection();
        testFieldTransformHeadingZero();
        testFieldTransformQuarterTurn();
        testStableTrack();
        testTwoNearbyPieces();
        testDisappearance();
        testMovingBall();
        testWrongType();
        testUnknownCameraId();
        testInvalidPose();
        testConfidenceMatures();
        testPlannerSeam();
        testFourInchPairStaysSplit();
        testClusterJitterKeepsIds();
        testInputOrderDoesNotSwapIds();
        testOneMissingFromCluster();
        testOverdeterminedCalibration();
        testHistoricalPoseWhileMoving();
        testConfiguredLatency();
        testZeroLatencyUsesObservationTime();
        testMissingHistoricalPose();
        testAllianceOwnership();
        System.out.println("PIECE TRACKER CHECKS PASSED (" + checks + ")");
    }

    private static void testProjection() {
        HomographyFloorProjection scale = HomographyFloorProjection.fromCoefficients(new double[] {
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 1.0
        });
        FloorPoint point = scale.project(100.0, 50.0);
        check(point != null, "A projection exists");
        near(point.getXForwardInches(), 10.0, "A image x maps to robot forward");
        near(point.getYLeftInches(), 5.0, "A image y maps to robot left");

        HomographyFloorProjection solved = HomographyFloorProjection.solve(
                new double[][] {{0, 0}, {100, 0}, {0, 100}, {100, 100}},
                new double[][] {{0, 0}, {10, 0}, {0, 10}, {10, 10}});
        check(solved.isConfigured(), "A four measured points configure a homography");
        FloorPoint solvedPoint = solved.project(200.0, 50.0);
        check(solvedPoint != null, "A solved homography projects");
        near(solvedPoint.getXForwardInches(), 20.0, "A solved homography keeps x");
        near(solvedPoint.getYLeftInches(), 5.0, "A solved homography keeps y");

        PieceTracker lower = new PieceTracker(scale, constants(ObservationAnchor.LOWER_CENTER, 6.0, 0.5, 1.0));
        PieceTrackingResult lowered = place(lower, 
                Arrays.asList(PieceObservation.of(1, 100.0, 40.0, 10.0, 20.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(lowered.getSightings().get(0).hasRobotPoint(), "A lower center projects");
        near(lowered.getSightings().get(0).getRobotY(), 5.0, "A lower center uses the bottom of the box");

        PieceTracker missing = new PieceTracker(HomographyFloorProjection.unconfigured(), constants(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0));
        PieceTrackingResult blank = place(missing, 
                Arrays.asList(pixel(1, 100.0, 50.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(!blank.isCalibrationConfigured(), "A missing calibration is reported");
        check(blank.getPieces().isEmpty(), "A missing calibration does not invent field pieces");
    }

    private static void testFieldTransformHeadingZero() {
        PieceTrackingResult result = projectForward(0.0);
        TrackedPiece piece = only(result);
        near(piece.getFieldX(), 70.0, "B heading 0 forward increases field X");
        near(piece.getFieldY(), 50.0, "B heading 0 does not change field Y");
        near(FieldPlacement.fieldX(50.0, 0.0, 20.0, 0.0), 70.0, "B formula matches heading 0");
    }

    private static void testFieldTransformQuarterTurn() {
        PieceTrackingResult result = projectForward(Math.PI / 2.0);
        TrackedPiece piece = only(result);
        near(piece.getFieldX(), 50.0, "C heading +90 forward does not change field X");
        near(piece.getFieldY(), 70.0, "C heading +90 forward increases field Y");
    }

    private static void testStableTrack() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 0.5);
        int id = -1;
        for (int frame = 0; frame < 10; frame++) {
            double jitter = (frame % 2 == 0) ? 0.0 : 0.4;
            PieceTrackingResult result = place(tracker, 
                    Arrays.asList(pixel(1, 30.0 + jitter, 0.0, frame)),
                    robot(0.0, 0.0, 0.0),
                    frame);
            check(result.getPieces().size() == 1, "D one track across jitter");
            int current = result.getPieces().get(0).getId();
            if (id < 0) {
                id = current;
            }
            check(current == id, "D track id is stable");
        }

        PieceTracker duplicates = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult doubled = place(duplicates, 
                Arrays.asList(pixel(1, 30.0, 0.0, 0.0), pixel(1, 31.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(doubled.getPieces().size() == 2, "D two detections in one frame start two tracks");
        check(doubled.getPieces().get(0).getId() != doubled.getPieces().get(1).getId(), "D those tracks have different ids");
    }

    private static void testTwoNearbyPieces() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult result = place(tracker, 
                Arrays.asList(pixel(1, 10.0, 0.0, 0.0), pixel(1, 40.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(result.getPieces().size() == 2, "E two separated balls stay two tracks");
        check(result.getPieces().get(0).getId() != result.getPieces().get(1).getId(), "E ids differ");
    }

    private static void testDisappearance() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult seen = place(tracker, 
                Arrays.asList(pixel(1, 12.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        double fresh = only(seen).getConfidence();
        PieceTrackingResult aged = place(tracker, 
                Arrays.asList(),
                robot(0.0, 0.0, 0.0),
                0.2);
        check(aged.getPieces().size() == 1, "F track remains before timeout");
        check(only(aged).getId() == only(seen).getId(), "F id survives the gap");
        check(only(aged).getConfidence() < fresh, "F confidence falls while it is missing");
        PieceTrackingResult gone = place(tracker, 
                Arrays.asList(),
                robot(0.0, 0.0, 0.0),
                0.51);
        check(gone.getPieces().isEmpty(), "F track is removed after timeout");
    }

    private static void testMovingBall() {
        PieceTracker tracker = tracker(identity(), 8.0, 0.5, 0.5);
        PieceTrackingResult first = place(tracker, 
                Arrays.asList(pixel(2, 10.0, 4.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        int id = only(first).getId();
        PieceTrackingResult moved = place(tracker, 
                Arrays.asList(pixel(2, 14.0, 4.0, 0.1)),
                robot(0.0, 0.0, 0.0),
                0.1);
        TrackedPiece piece = only(moved);
        check(piece.getId() == id, "G moved ball keeps its id");
        near(piece.getFieldX(), 12.0, "G position blends toward the new observation");
        check(piece.getType() == BallType.NECTAR, "G type stays nectar");
    }

    private static void testWrongType() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult nectar = place(tracker, 
                Arrays.asList(pixel(2, 10.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        int nectarId = only(nectar).getId();
        PieceTrackingResult both = place(tracker, 
                Arrays.asList(pixel(1, 10.0, 0.0, 0.1)),
                robot(0.0, 0.0, 0.0),
                0.1);
        check(both.getPieces().size() == 2, "H pollen does not replace the nectar track");
        TrackedPiece stillNectar = find(both.getPieces(), nectarId);
        check(stillNectar != null && stillNectar.getType() == BallType.NECTAR, "H nectar track keeps its type");
        check(countType(both.getPieces(), BallType.POLLEN) == 1, "H pollen is its own track");
    }

    private static void testUnknownCameraId() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult result = place(tracker, 
                Arrays.asList(pixel(99, 10.0, 0.0, 0.0), pixel(1, 20.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(result.getIgnoredUnknownIds() == 1, "I unknown id is ignored");
        check(result.getPieces().size() == 1, "I known id still tracks");
        check(only(result).getType() == BallType.POLLEN, "I remaining track is pollen");
    }

    private static void testInvalidPose() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult result = place(tracker, 
                Arrays.asList(pixel(1, 15.0, 3.0, 0.0)),
                RobotState.invalid(0.0),
                0.0);
        check(result.getPieces().isEmpty(), "J invalid pose creates no planner pieces");
        check(!result.isPoseUsed(), "J pose is not used");
        check(result.getSightings().size() == 1, "J the camera read is still available");
        check(result.getSightings().get(0).hasRobotPoint(), "J robot-floor inches do not require a field pose");
    }

    private static void testConfidenceMatures() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        double first = 0.0;
        double last = 0.0;
        for (int frame = 0; frame < 5; frame++) {
            PieceTrackingResult result = place(tracker, 
                    Arrays.asList(pixel(1, 8.0, 1.0, frame * 0.05)),
                    robot(0.0, 0.0, 0.0),
                    frame * 0.05);
            last = only(result).getConfidence();
            if (frame == 0) {
                first = last;
            }
        }
        check(first < 0.35, "K a new track is low confidence");
        check(last > 0.95, "K repeated hits raise confidence");
        check(last > first, "K confidence increased");
    }

    private static void testPlannerSeam() {
        PieceTracker tracker = new PieceTracker(identity(), configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0, AllianceColor.RED, 0.0, true));
        PieceTrackingResult result = place(tracker,
                Arrays.asList(pixel(2, 24.0, 0.0, 1.0)),
                robot(0.0, 0.0, 0.0),
                1.0);
        TrackedPiece piece = only(result);
        check(piece.getType() == BallType.NECTAR, "L red nectar stays nectar");
        check(piece.getOwnership() == PieceOwnership.ALLIANCE, "L red alliance owns red nectar");
        check(piece.isCollectable(), "L our nectar is collectable");
        TipModel tips = new TipModel() {
            @Override
            public double getTipProbability(BallLoad load) {
                return load.getNectar() > 0 ? 0.9 : 0.0;
            }
        };
        PickupPlanner planner = new PickupPlanner(
                tips,
                new EuclideanTravelTimeModel(),
                new FixedShotSetupModel(1.0));
        PickupPlan plan = planner.plan(robot(0.0, 0.0, 0.0), BallLoad.empty(), result.getPieces(), 1.0, null);
        check(plan.getDecision() == PickupPlan.Decision.PICKUP, "L planner accepts tracker pieces");
        check(plan.getTargets().size() == 1, "L planner keeps the one piece");
        check(plan.getTargets().get(0).getPiece().getId() == piece.getId(), "L planner uses the stable track id");
    }

    private static void testFourInchPairStaysSplit() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult created = place(tracker, 
                Arrays.asList(pixel(1, 0.0, 0.0, 0.0), pixel(1, 4.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(created.getPieces().size() == 2, "two balls 4 inches apart start two tracks");
        int left = nearestX(created.getPieces(), 0.0).getId();
        int right = nearestX(created.getPieces(), 4.0).getId();
        check(left != right, "the 4 inch pair has two ids");
        PieceTrackingResult again = place(tracker, 
                Arrays.asList(pixel(1, 4.1, 0.0, 0.05), pixel(1, 0.1, 0.0, 0.05)),
                robot(0.0, 0.0, 0.0),
                0.05);
        check(again.getPieces().size() == 2, "the 4 inch pair stays two tracks");
        check(nearestX(again.getPieces(), 0.0).getId() == left, "left ball keeps its id inside the gate");
        check(nearestX(again.getPieces(), 4.0).getId() == right, "right ball keeps its id inside the gate");
    }

    private static void testClusterJitterKeepsIds() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        double[][] frames = new double[][] {
                {0.0, 4.0, 8.0},
                {0.4, 3.7, 8.3},
                {-0.3, 4.4, 7.6},
                {0.2, 4.1, 8.2}
        };
        int[] ids = null;
        for (int frame = 0; frame < frames.length; frame++) {
            PieceTrackingResult result = place(tracker, 
                    Arrays.asList(
                            pixel(1, frames[frame][0], 0.0, frame * 0.05),
                            pixel(1, frames[frame][1], 0.0, frame * 0.05),
                            pixel(1, frames[frame][2], 0.0, frame * 0.05)),
                    robot(0.0, 0.0, 0.0),
                    frame * 0.05);
            check(result.getPieces().size() == 3, "cluster jitter keeps three tracks");
            if (ids == null) {
                ids = new int[] {
                        nearestX(result.getPieces(), 0.0).getId(),
                        nearestX(result.getPieces(), 4.0).getId(),
                        nearestX(result.getPieces(), 8.0).getId()
                };
            }
            check(nearestX(result.getPieces(), frames[frame][0]).getId() == ids[0], "left cluster id holds");
            check(nearestX(result.getPieces(), frames[frame][1]).getId() == ids[1], "middle cluster id holds");
            check(nearestX(result.getPieces(), frames[frame][2]).getId() == ids[2], "right cluster id holds");
        }
    }

    private static void testInputOrderDoesNotSwapIds() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult created = place(tracker, 
                Arrays.asList(pixel(1, 0.0, 0.0, 0.0), pixel(1, 4.0, 0.0, 0.0), pixel(1, 8.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        int left = nearestX(created.getPieces(), 0.0).getId();
        int middle = nearestX(created.getPieces(), 4.0).getId();
        int right = nearestX(created.getPieces(), 8.0).getId();
        PieceTrackingResult reversed = place(tracker, 
                Arrays.asList(pixel(1, 7.8, 0.0, 0.05), pixel(1, 0.2, 0.0, 0.05), pixel(1, 4.1, 0.0, 0.05)),
                robot(0.0, 0.0, 0.0),
                0.05);
        check(reversed.getPieces().size() == 3, "reversed input still has three tracks");
        check(nearestX(reversed.getPieces(), 0.2).getId() == left, "input order does not move the left id");
        check(nearestX(reversed.getPieces(), 4.1).getId() == middle, "input order does not move the middle id");
        check(nearestX(reversed.getPieces(), 7.8).getId() == right, "input order does not move the right id");
    }

    private static void testOneMissingFromCluster() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult created = place(tracker, 
                Arrays.asList(pixel(1, 0.0, 0.0, 0.0), pixel(1, 4.0, 0.0, 0.0), pixel(1, 8.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        int left = nearestX(created.getPieces(), 0.0).getId();
        int middle = nearestX(created.getPieces(), 4.0).getId();
        int right = nearestX(created.getPieces(), 8.0).getId();
        PieceTrackingResult missing = place(tracker, 
                Arrays.asList(pixel(1, 0.2, 0.0, 0.1), pixel(1, 7.9, 0.0, 0.1)),
                robot(0.0, 0.0, 0.0),
                0.1);
        check(missing.getPieces().size() == 3, "a brief miss keeps the third track");
        check(nearestX(missing.getPieces(), 0.2).getId() == left, "left id survives the missing neighbor");
        check(nearestX(missing.getPieces(), 7.9).getId() == right, "right id survives the missing neighbor");
        TrackedPiece aged = find(missing.getPieces(), middle);
        check(aged != null, "the missing ball keeps its own id");
        check(Math.abs(aged.getFieldX() - 4.0) < 1.0e-6, "the missing ball is not pulled onto a neighbor");
    }

    private static void testOverdeterminedCalibration() {
        double[][] fourImage = new double[][] {{0, 0}, {100, 0}, {0, 100}, {100, 100}};
        double[][] fourFloor = new double[][] {{0, 0}, {10, 0}, {0, 10}, {10, 10}};
        HomographyFloorProjection.Calibration exact = HomographyFloorProjection.calibrate(fourImage, fourFloor);
        check(exact.isConfigured(), "four samples still configure");
        FloorPoint exactPoint = exact.getProjection().project(200.0, 50.0);
        near(exactPoint.getXForwardInches(), 20.0, "four-sample calibrate matches the exact solver in x");
        near(exactPoint.getYLeftInches(), 5.0, "four-sample calibrate matches the exact solver in y");
        check(exact.getRmsErrorInches() < 1.0e-6, "exact four-point residual is near zero");
        check(exact.getMaxErrorInches() < 1.0e-6, "exact four-point max error is near zero");

        double[][] manyImage = new double[][] {
                {0, 0}, {100, 0}, {0, 100}, {100, 100}, {50, 50}, {80, 20}, {20, 70}
        };
        double[][] manyFloor = new double[][] {
                {0, 0}, {10, 0}, {0, 10}, {10, 10}, {5, 5}, {8, 2}, {2, 7}
        };
        HomographyFloorProjection.Calibration fitted = HomographyFloorProjection.calibrate(manyImage, manyFloor);
        check(fitted.isConfigured(), "more than four samples configure");
        check(fitted.getSampleCount() == 7, "all samples are counted");
        FloorPoint fittedPoint = fitted.getProjection().project(200.0, 50.0);
        near(fittedPoint.getXForwardInches(), 20.0, "overdetermined fit keeps x");
        near(fittedPoint.getYLeftInches(), 5.0, "overdetermined fit keeps y");
        check(fitted.getRmsErrorInches() < 1.0e-4, "consistent extra samples have a small RMS");
        check(fitted.getMaxErrorInches() < 1.0e-4, "consistent extra samples have a small max error");
        check(fitted.getMaxErrorInches() + 1.0e-12 >= fitted.getRmsErrorInches(), "max error is at least the RMS");

        double[][] noisyFloor = new double[][] {
                {0, 0}, {10, 0}, {0, 10}, {10, 10}, {5, 5}, {9, 2}, {2, 7}
        };
        HomographyFloorProjection.Calibration noisy = HomographyFloorProjection.calibrate(manyImage, noisyFloor);
        check(noisy.isConfigured(), "a noisy extra sample still fits");
        check(noisy.getRmsErrorInches() > 0.05, "RMS reports the noisy sample");
        check(noisy.getMaxErrorInches() >= noisy.getRmsErrorInches(), "max error covers the worst sample");

        HomographyFloorProjection.Calibration tooFew = HomographyFloorProjection.calibrate(
                new double[][] {{0, 0}, {1, 0}, {0, 1}},
                new double[][] {{0, 0}, {1, 0}, {0, 1}});
        check(!tooFew.isConfigured(), "fewer than four samples stay unconfigured");
        check(!Double.isFinite(tooFew.getRmsErrorInches()), "an unconfigured fit does not report zero RMS");
    }

    /**
     * 50 ms is a test fixture for a moving robot, not a measured HuskyLens delay.
     * The production latency constant stays 0.
     */
    private static void testHistoricalPoseWhileMoving() {
        PieceTrackerConstants knobs = configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0, AllianceColor.UNKNOWN, 0.0, true);
        RobotStateHistory history = new RobotStateHistory();
        history.add(moving(0.95, 38.0));
        history.add(moving(1.00, 40.0));
        PieceTracker historical = new PieceTracker(identity(), knobs);
        PieceTrackingResult past = historical.update(
                Arrays.asList(pixel(1, 10.0, 0.0, 0.95)),
                history,
                1.00);
        near(only(past).getFieldX(), 48.0, "F the piece uses the pose from 50 ms earlier");
        check(past.getSightings().get(0).hasHistoricalPose(), "F historical pose is found");

        RobotStateHistory liveHistory = new RobotStateHistory();
        liveHistory.add(moving(1.00, 40.0));
        PieceTracker liveTracker = new PieceTracker(identity(), knobs);
        PieceTrackingResult live = liveTracker.update(
                Arrays.asList(pixel(1, 10.0, 0.0, 1.00)),
                liveHistory,
                1.00);
        near(only(live).getFieldX(), 50.0, "F the current pose is two inches farther along +X");
        near(Math.abs(only(live).getFieldX() - only(past).getFieldX()), 2.0,
                "F history and the current pose disagree by about 2 inches");
    }

    private static void testConfiguredLatency() {
        PieceTracker tracker = new PieceTracker(identity(), configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0, AllianceColor.UNKNOWN, 0.100, true));
        RobotStateHistory history = new RobotStateHistory();
        history.add(moving(10.000, 0.0));
        history.add(moving(10.100, 4.0));
        PieceTrackingResult result = tracker.update(
                Arrays.asList(pixel(1, 10.0, 0.0, 10.100)),
                history,
                10.100);
        PieceTrackingResult.Sighting sighting = result.getSightings().get(0);
        check(Math.abs(sighting.getReadTimestampSec() - 10.100) < 1.0e-9, "G the read time is 10.100");
        check(Math.abs(sighting.getCaptureTimestampSec() - 10.000) < 1.0e-6,
                "G capture time is the read time minus the test latency");
        check(sighting.hasHistoricalPose(), "G a pose exists at the capture time");
        near(sighting.getHistoricalX(), 0.0, "G the pose is the one near 10.000, not 10.100");
        near(only(result).getFieldX(), 10.0, "G the field piece follows the capture pose");
    }

    private static void testZeroLatencyUsesObservationTime() {
        PieceTracker tracker = new PieceTracker(identity(), configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0,
                AllianceColor.UNKNOWN, PieceTrackerConstants.HUSKYLENS_LATENCY_SEC, true));
        RobotStateHistory history = new RobotStateHistory();
        history.add(moving(5.0, 12.0));
        history.add(moving(5.2, 20.0));
        PieceTrackingResult result = tracker.update(
                Arrays.asList(pixel(1, 0.0, 0.0, 5.0)),
                history,
                5.2);
        PieceTrackingResult.Sighting sighting = result.getSightings().get(0);
        near(sighting.getCaptureTimestampSec(), sighting.getReadTimestampSec(),
                "H zero latency uses the observation timestamp");
        near(only(result).getFieldX(), 12.0, "H the piece uses the observation-time pose");
    }

    private static void testMissingHistoricalPose() {
        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult result = tracker.update(
                Arrays.asList(pixel(1, 8.0, 1.0, 1.0)),
                new RobotStateHistory(),
                1.0);
        check(result.getSightings().size() == 1, "I the raw detection is still reported");
        check(result.getSightings().get(0).hasRobotPoint(), "I robot-floor inches still project");
        check(!result.getSightings().get(0).hasHistoricalPose(), "I no historical pose is invented");
        check(result.getPieces().isEmpty(), "I no planner piece is published");
        check(!result.isPoseUsed(), "I field placement is withheld");
    }

    private static void testAllianceOwnership() {
        TrackedPiece redOnRed = classified(AllianceColor.RED, 2);
        check(redOnRed.getType() == BallType.NECTAR, "J red nectar is nectar");
        check(redOnRed.getOwnership() == PieceOwnership.ALLIANCE, "J red alliance owns red nectar");
        check(redOnRed.isCollectable(), "J our nectar is collectable");

        TrackedPiece blueOnRed = classified(AllianceColor.RED, 3);
        check(blueOnRed.getType() == BallType.NECTAR, "K blue nectar is nectar");
        check(blueOnRed.getOwnership() == PieceOwnership.OPPONENT, "K red alliance treats blue nectar as theirs");
        check(!blueOnRed.isCollectable(), "K their nectar is not collectable");

        TrackedPiece blueOnBlue = classified(AllianceColor.BLUE, 3);
        check(blueOnBlue.getOwnership() == PieceOwnership.ALLIANCE, "L blue alliance owns blue nectar");
        check(blueOnBlue.isCollectable(), "L our nectar is collectable on blue");
        TrackedPiece redOnBlue = classified(AllianceColor.BLUE, 2);
        check(redOnBlue.getOwnership() == PieceOwnership.OPPONENT, "L blue alliance treats red nectar as theirs");
        check(!redOnBlue.isCollectable(), "L their nectar is not collectable on blue");

        TrackedPiece unknownNectar = classified(AllianceColor.UNKNOWN, 2);
        check(unknownNectar.getType() == BallType.NECTAR, "M unknown alliance still sees nectar");
        check(unknownNectar.getOwnership() == PieceOwnership.UNKNOWN, "M nectar ownership stays unknown");
        check(!unknownNectar.isCollectable(), "M unknown nectar is not collectable");

        TrackedPiece pollen = classified(AllianceColor.UNKNOWN, 1);
        check(pollen.getType() == BallType.POLLEN, "N pollen stays pollen");
        check(pollen.getOwnership() == PieceOwnership.NEUTRAL, "N pollen stays neutral");
        check(pollen.isCollectable(), "N neutral pollen follows the collectability flag");

        PieceTracker withheld = new PieceTracker(identity(), configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0, AllianceColor.UNKNOWN, 0.0, false));
        PieceTrackingResult held = place(withheld,
                Arrays.asList(pixel(1, 4.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(only(held).getOwnership() == PieceOwnership.NEUTRAL, "N pollen ownership does not depend on the flag");
        check(!only(held).isCollectable(), "N pollen collectability stays configurable");

        PieceTracker tracker = tracker(identity(), 6.0, 0.5, 1.0);
        PieceTrackingResult unknownId = place(tracker,
                Arrays.asList(pixel(9, 4.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        check(unknownId.getPieces().isEmpty(), "O an unknown camera id is ignored");
        check(unknownId.getIgnoredUnknownIds() == 1, "O the ignored id is counted");
        check(unknownId.getSightings().isEmpty(), "O an unknown id is not pollen or nectar");
    }

    private static TrackedPiece classified(AllianceColor alliance, int cameraId) {
        PieceTracker tracker = new PieceTracker(identity(), configured(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0, alliance, 0.0, true));
        PieceTrackingResult result = place(tracker,
                Arrays.asList(pixel(cameraId, 4.0, 0.0, 0.0)),
                robot(0.0, 0.0, 0.0),
                0.0);
        return only(result);
    }

    private static RobotState moving(double time, double x) {
        return new RobotState(
                time,
                x, 0.0, 0.0,
                40.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                true,
                true);
    }

    private static TrackedPiece nearestX(List<TrackedPiece> pieces, double x) {
        TrackedPiece best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < pieces.size(); i++) {
            double distance = Math.abs(pieces.get(i).getFieldX() - x);
            if (distance < bestDistance) {
                best = pieces.get(i);
                bestDistance = distance;
            }
        }
        return best;
    }

    private static PieceTrackingResult projectForward(double heading) {
        HomographyFloorProjection forward = HomographyFloorProjection.fromCoefficients(new double[] {
                0.0, 0.0, 20.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0
        });
        PieceTracker tracker = new PieceTracker(forward, constants(
                ObservationAnchor.BOUNDING_BOX_CENTER, 6.0, 0.5, 1.0));
        return place(tracker, 
                Arrays.asList(pixel(1, 0.0, 0.0, 0.0)),
                robot(50.0, 50.0, heading),
                0.0);
    }

    private static PieceTracker tracker(
            HomographyFloorProjection projection,
            double gate,
            double timeout,
            double blend) {
        return new PieceTracker(projection, constants(
                ObservationAnchor.BOUNDING_BOX_CENTER, gate, timeout, blend));
    }

    private static PieceTrackerConstants constants(
            ObservationAnchor anchor,
            double gate,
            double timeout,
            double blend) {
        return configured(anchor, gate, timeout, blend, AllianceColor.UNKNOWN, 0.0, true);
    }

    private static PieceTrackerConstants configured(
            ObservationAnchor anchor,
            double gate,
            double timeout,
            double blend,
            AllianceColor alliance,
            double latencySec,
            boolean collectNeutralPollen) {
        return new PieceTrackerConstants(
                1, 2, 3, alliance, latencySec, gate, timeout, blend, 5.0, anchor, collectNeutralPollen);
    }

    /**
     * Existing scenarios stored the pose on a {@link RobotState} whose timestamp
     * was 0. Latency is 0, and those observations use {@code now} as their
     * timestamp, so the history sample is stamped at {@code now}.
     */
    private static PieceTrackingResult place(
            PieceTracker tracker,
            List<PieceObservation> observations,
            RobotState pose,
            double now) {
        RobotStateHistory history = new RobotStateHistory();
        history.add(at(pose, now));
        return tracker.update(observations, history, now);
    }

    private static RobotState at(RobotState pose, double time) {
        return new RobotState(
                time,
                pose.getFieldX(), pose.getFieldY(), pose.getHeadingRad(),
                pose.getFieldVx(), pose.getFieldVy(), pose.getAngularVelocityRadPerSec(),
                pose.getFieldAx(), pose.getFieldAy(), pose.getAngularAccelerationRadPerSec2(),
                pose.isPoseValid(),
                pose.isVelocityValid());
    }

    private static HomographyFloorProjection identity() {
        return HomographyFloorProjection.fromCoefficients(new double[] {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
        });
    }

    private static PieceObservation pixel(int cameraId, double x, double y, double time) {
        return PieceObservation.of(cameraId, x, y, 0.0, 0.0, time);
    }

    private static RobotState robot(double x, double y, double heading) {
        return new RobotState(
                0.0,
                x, y, heading,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                true,
                true);
    }

    private static TrackedPiece only(PieceTrackingResult result) {
        check(result.getPieces().size() == 1, "expected one piece");
        return result.getPieces().get(0);
    }

    private static TrackedPiece find(List<TrackedPiece> pieces, int id) {
        for (int i = 0; i < pieces.size(); i++) {
            if (pieces.get(i).getId() == id) {
                return pieces.get(i);
            }
        }
        return null;
    }

    private static int countType(List<TrackedPiece> pieces, BallType type) {
        int count = 0;
        for (int i = 0; i < pieces.size(); i++) {
            if (pieces.get(i).getType() == type) {
                count++;
            }
        }
        return count;
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
