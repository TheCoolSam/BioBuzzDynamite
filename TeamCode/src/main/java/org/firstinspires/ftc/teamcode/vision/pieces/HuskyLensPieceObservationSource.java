package org.firstinspires.ftc.teamcode.vision.pieces;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * HuskyLens blocks to {@link PieceObservation}. This is the only perception
 * class that mentions the device.
 *
 * <p>FTC 12.0 {@link HuskyLens#blocks()} returns at most six blocks. Each
 * block has integer {@code id}, center {@code x}/{@code y}, and
 * {@code width}/{@code height}. The driver has no confidence field. A failed
 * read returns an empty list instead of throwing into the OpMode.
 *
 * <p>The algorithm is {@link HuskyLens.Algorithm#OBJECT_RECOGNITION} because
 * pollen and nectar are taught objects with separate ids. Object tracking
 * follows one target. Line tracking returns arrows, which this source ignores.
 */
public final class HuskyLensPieceObservationSource implements PieceObservationSource {

    private final HuskyLens huskyLens;

    public HuskyLensPieceObservationSource(HuskyLens huskyLens) {
        this.huskyLens = huskyLens;
    }

    /**
     * Asks the device for object recognition. Returns false when the device
     * is missing or the call fails. The SDK does not set an algorithm itself.
     */
    public boolean selectObjectRecognition() {
        if (huskyLens == null) {
            return false;
        }
        try {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
            return true;
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    /** {@link HuskyLens#knock()}. False when the device does not answer. */
    public boolean knock() {
        if (huskyLens == null) {
            return false;
        }
        try {
            return huskyLens.knock();
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    @Override
    public List<PieceObservation> read(double timestampSec) {
        if (huskyLens == null) {
            return Collections.emptyList();
        }
        HuskyLens.Block[] blocks;
        try {
            blocks = huskyLens.blocks();
        } catch (RuntimeException ignored) {
            return Collections.emptyList();
        }
        if (blocks == null || blocks.length == 0) {
            return Collections.emptyList();
        }
        List<PieceObservation> observations = new ArrayList<PieceObservation>();
        for (int i = 0; i < blocks.length; i++) {
            HuskyLens.Block block = blocks[i];
            if (block == null) {
                continue;
            }
            observations.add(PieceObservation.of(
                    block.id,
                    block.x,
                    block.y,
                    block.width,
                    block.height,
                    timestampSec));
        }
        return observations;
    }
}
