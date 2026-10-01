package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotStateHistory;
import org.firstinspires.ftc.teamcode.vision.pieces.HomographyFloorProjection;
import org.firstinspires.ftc.teamcode.vision.pieces.HuskyLensPieceObservationSource;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceObservation;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTracker;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackerConstants;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackingResult;

import java.util.List;

/**
 * Reads the HuskyLens and prints tracks. Does not power any motor.
 *
 * <p>This OpMode does not own localization, so the history stays empty and
 * field pieces stay withheld. The latency lines are here so a later measured
 * delay can be compared against the robot pose once a localizer is connected.
 * Camera ids 1, 2, and 3 are placeholders until yellow, red, and blue are
 * taught in Color Recognition with Learn Multiple enabled on the device.
 */
@TeleOp(name = "HuskyLens Piece Test", group = "Test")
public class HuskyLensPieceTestOpMode extends OpMode {

    private final ElapsedTime timer = new ElapsedTime();
    private final RobotStateHistory history = new RobotStateHistory();
    private HuskyLensPieceObservationSource source;
    private PieceTracker tracker;
    private PieceTrackerConstants constants;
    private boolean cameraReady;
    private String failure = "";

    @Override
    public void init() {
        timer.reset();
        constants = PieceTrackerConstants.defaults();
        tracker = new PieceTracker(HomographyFloorProjection.unconfigured(), constants);
        try {
            HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            source = new HuskyLensPieceObservationSource(huskyLens);
            boolean alive = source.knock();
            boolean selected = source.selectColorRecognition();
            cameraReady = alive && selected;
            if (!cameraReady) {
                failure = alive ? "algorithm not selected" : "knock failed";
            }
        } catch (RuntimeException e) {
            cameraReady = false;
            failure = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        telemetry.addData("Camera", cameraReady ? "ready" : "not ready");
        telemetry.addLine("Color Recognition: teach yellow/red/blue; enable Learn Multiple.");
        telemetry.addLine("Firmware 0.5.1+ required for multiple blocks of the same color.");
        telemetry.addLine("No drivetrain commands in this OpMode.");
    }

    @Override
    public void loop() {
        double now = timer.seconds();
        List<PieceObservation> observations = source == null
                ? java.util.Collections.<PieceObservation>emptyList()
                : source.read(now);
        PieceTrackingResult result = tracker.update(observations, history, now);

        telemetry.addData("Camera", cameraReady ? "ready" : failure);
        telemetry.addData("Camera latency s", constants.huskyLensLatencySec);
        telemetry.addData("Raw detections", observations.size());
        for (int i = 0; i < observations.size(); i++) {
            PieceObservation observation = observations.get(i);
            telemetry.addData(
                    "Raw " + i,
                    "id %d  x %.0f  y %.0f  read %.3f",
                    observation.getCameraId(),
                    observation.getImageX(),
                    observation.getImageY(),
                    observation.getTimestampSec());
        }
        if (!result.isCalibrationConfigured()) {
            telemetry.addLine("FLOOR CALIBRATION NOT CONFIGURED");
        }
        telemetry.addLine("Current robot pose: not supplied");
        List<PieceTrackingResult.Sighting> sightings = result.getSightings();
        for (int i = 0; i < sightings.size(); i++) {
            PieceTrackingResult.Sighting sighting = sightings.get(i);
            telemetry.addData("Read " + i, "%.3f", sighting.getReadTimestampSec());
            telemetry.addData("Capture " + i, "%.3f", sighting.getCaptureTimestampSec());
            telemetry.addData("Historical pose " + i, sighting.hasHistoricalPose() ? "YES" : "NO");
            if (sighting.hasHistoricalPose()) {
                telemetry.addData(
                        "Historical robot " + i,
                        "x %.1f  y %.1f  h %.3f",
                        sighting.getHistoricalX(),
                        sighting.getHistoricalY(),
                        sighting.getHistoricalHeadingRad());
            }
            if (sighting.hasRobotPoint()) {
                telemetry.addData(
                        "Floor " + i,
                        "%s  robot (%.1f, %.1f)",
                        sighting.getType(),
                        sighting.getRobotX(),
                        sighting.getRobotY());
            }
        }
        telemetry.addData("Unknown ids ignored", result.getIgnoredUnknownIds());
        List<TrackedPiece> pieces = result.getPieces();
        telemetry.addData("Planner pieces", pieces.size());
        for (int i = 0; i < pieces.size(); i++) {
            TrackedPiece piece = pieces.get(i);
            telemetry.addData(
                    "Track " + piece.getId(),
                    "%s  %s  collect %s  field (%.1f, %.1f)",
                    piece.getType(),
                    piece.getOwnership(),
                    piece.isCollectable() ? "YES" : "NO",
                    piece.getFieldX(),
                    piece.getFieldY());
        }
    }
}
