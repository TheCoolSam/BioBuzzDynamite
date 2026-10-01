package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.List;

/**
 * Reads the current camera frame as pixel observations.
 * Implementations may talk to a device. Callers must not.
 */
public interface PieceObservationSource {

    /**
     * @param timestampSec time of this read, in seconds
     * @return observations from this read, or an empty list when the camera fails
     */
    List<PieceObservation> read(double timestampSec);
}
