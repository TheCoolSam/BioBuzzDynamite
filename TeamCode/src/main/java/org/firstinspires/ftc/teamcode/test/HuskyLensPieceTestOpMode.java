package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.vision.pieces.HuskyLensPieceObservationSource;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceObservation;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTracker;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackerConstants;
import org.firstinspires.ftc.teamcode.vision.pieces.PieceTrackingResult;
import org.firstinspires.ftc.teamcode.vision.pieces.HomographyFloorProjection;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;

import java.util.List;

/**
 * Reads the HuskyLens and prints tracks. Does not power any motor.
 *
 * <p>This OpMode does not own localization. Field pieces are withheld because
 * the pose passed in is invalid on purpose. Robot-floor inches appear only
 * after measured homography coefficients replace the unconfigured projection.
 */
@TeleOp(name = "HuskyLens Piece Test", group = "Test")
public class HuskyLensPieceTestOpMode extends OpMode {

    private final ElapsedTime timer = new ElapsedTime();
    private HuskyLensPieceObservationSource source;
    private PieceTracker tracker;
    private boolean cameraReady;
    private String failure = "";

    @Override
    public void init() {
        timer.reset();
        tracker = new PieceTracker(HomographyFloorProjection.unconfigured(), PieceTrackerConstants.defaults());
        try {
            HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            source = new HuskyLensPieceObservationSource(huskyLens);
            boolean alive = source.knock();
            boolean selected = source.selectObjectRecognition();
            cameraReady = alive && selected;
            if (!cameraReady) {
                failure = alive ? "algorithm not selected" : "knock failed";
            }
        } catch (RuntimeException e) {
            cameraReady = false;
            failure = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
        telemetry.addData("Camera", cameraReady ? "ready" : "not ready");
        telemetry.addLine("No drivetrain commands in this OpMode.");
    }

    @Override
    public void loop() {
        double now = timer.seconds();
        List<PieceObservation> observations = source == null
                ? java.util.Collections.<PieceObservation>emptyList()
                : source.read(now);
        PieceTrackingResult result = tracker.update(observations, RobotState.invalid(now), now);

        telemetry.addData("Camera", cameraReady ? "ready" : failure);
        telemetry.addData("Raw detections", observations.size());
        for (int i = 0; i < observations.size(); i++) {
            PieceObservation observation = observations.get(i);
            telemetry.addData(
                    "Raw " + i,
                    "id %d  x %.0f  y %.0f",
                    observation.getCameraId(),
                    observation.getImageX(),
                    observation.getImageY());
        }
        if (!result.isCalibrationConfigured()) {
            telemetry.addLine("FLOOR CALIBRATION NOT CONFIGURED");
        }
        telemetry.addData("Pose", result.isPoseUsed() ? "used" : "invalid, field tracks withheld");
        telemetry.addData("Unknown ids ignored", result.getIgnoredUnknownIds());
        List<TrackedPiece> pieces = result.getPieces();
        telemetry.addData("Planner pieces", pieces.size());
        for (int i = 0; i < pieces.size(); i++) {
            TrackedPiece piece = pieces.get(i);
            telemetry.addData(
                    "Track " + piece.getId(),
                    "%s  field (%.1f, %.1f)  conf %.2f  seen %.2f",
                    piece.getType(),
                    piece.getFieldX(),
                    piece.getFieldY(),
                    piece.getConfidence(),
                    now - piece.getLastSeenTimestampSec());
        }
        List<PieceTrackingResult.Sighting> sightings = result.getSightings();
        for (int i = 0; i < sightings.size(); i++) {
            PieceTrackingResult.Sighting sighting = sightings.get(i);
            if (sighting.hasRobotPoint()) {
                telemetry.addData(
                        "Floor " + i,
                        "%s  robot (%.1f, %.1f)",
                        sighting.getType(),
                        sighting.getRobotX(),
                        sighting.getRobotY());
            }
        }
    }
}
