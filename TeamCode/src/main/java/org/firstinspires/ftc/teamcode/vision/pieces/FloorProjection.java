package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * Maps an image pixel onto the floor in robot inches.
 * The mapping is replaceable. Pixel coordinates are not field coordinates.
 */
public interface FloorProjection {

    /** False until measured coefficients have been supplied. */
    boolean isConfigured();

    /**
     * @return robot-relative floor point, or null when this projection
     *         cannot answer (not configured, or the pixel is unusable)
     */
    FloorPoint project(double imageX, double imageY);
}
