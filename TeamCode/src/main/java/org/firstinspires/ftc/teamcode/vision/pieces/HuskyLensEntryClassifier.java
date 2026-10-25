package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.robot.RobotHardwareConfig;

/**
 * Optical entry classifier using a forward- or downward-angled HuskyLens mounted
 * near the intake mouth.
 *
 * <p>When a ball passes through the intake beam break, this classifier inspects the
 * latest HuskyLens color blocks. Blocks located near the bottom of the camera frame
 * (or closest to the intake entrance) determine the ball type (Pollen or Nectar)
 * and preserve entry FIFO ordering.
 */
public final class HuskyLensEntryClassifier implements RobotHardwareConfig.EntryClassifier {

    private final PieceTrackerConstants constants;
    private final double minIntakeImageY;
    private volatile List<PieceObservation> latestObservations = Collections.emptyList();

    /**
     * @param constants Color recognition ID mappings.
     * @param minIntakeImageY Minimum image Y coordinate (0-240) considered inside or entering the intake throat.
     */
    public HuskyLensEntryClassifier(PieceTrackerConstants constants, double minIntakeImageY) {
        this.constants = constants;
        this.minIntakeImageY = minIntakeImageY;
    }

    public HuskyLensEntryClassifier(PieceTrackerConstants constants) {
        this(constants, 140.0);
    }

    /** Called on each cycle by RevRobotIO with fresh HuskyLens observations. */
    public void updateObservations(List<PieceObservation> observations) {
        this.latestObservations = observations != null ? observations : Collections.<PieceObservation>emptyList();
    }

    @Override
    public BallType classify() {
        List<PieceObservation> obs = latestObservations;
        if (obs == null || obs.isEmpty() || constants == null) {
            return null;
        }

        // Look for the block closest to the intake throat (largest Y coordinate in image frame)
        PieceObservation bestCandidate = null;
        double maxCandidateY = -1;

        for (int i = 0; i < obs.size(); i++) {
            PieceObservation o = obs.get(i);
            if (o == null || !o.isValid()) continue;

            if (o.getImageY() >= minIntakeImageY) {
                if (o.getImageY() > maxCandidateY) {
                    maxCandidateY = o.getImageY();
                    bestCandidate = o;
                }
            }
        }

        // Fallback: If no block is strictly past minIntakeImageY, take the nearest detected block
        if (bestCandidate == null) {
            for (int i = 0; i < obs.size(); i++) {
                PieceObservation o = obs.get(i);
                if (o != null && o.isValid() && o.getImageY() > maxCandidateY) {
                    maxCandidateY = o.getImageY();
                    bestCandidate = o;
                }
            }
        }

        if (bestCandidate == null) {
            return null;
        }

        PieceTrackerConstants.ClassifiedPiece cp = constants.classify(bestCandidate.getCameraId());
        return cp != null ? cp.getType() : null;
    }
}
