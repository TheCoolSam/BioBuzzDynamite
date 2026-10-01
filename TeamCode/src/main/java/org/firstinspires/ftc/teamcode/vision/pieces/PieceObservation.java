package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * One camera detection in image pixels. This object does not know which
 * camera produced it, and it is not a field position.
 *
 * <p>Image X and Y are pixels. For the HuskyLens, the SDK documents a 320 by
 * 240 image whose {@code x} and {@code y} are the box center, with Y increasing
 * downward ({@code top = y - height / 2}).
 */
public final class PieceObservation {

    private final int cameraId;
    private final double imageX;
    private final double imageY;
    private final double imageWidth;
    private final double imageHeight;
    private final double timestampSec;
    private final boolean valid;

    private PieceObservation(
            int cameraId,
            double imageX,
            double imageY,
            double imageWidth,
            double imageHeight,
            double timestampSec,
            boolean valid) {
        this.cameraId = cameraId;
        this.imageX = imageX;
        this.imageY = imageY;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.timestampSec = timestampSec;
        this.valid = valid;
    }

    public static PieceObservation of(
            int cameraId,
            double imageX,
            double imageY,
            double imageWidth,
            double imageHeight,
            double timestampSec) {
        boolean finite = Double.isFinite(imageX)
                && Double.isFinite(imageY)
                && Double.isFinite(imageWidth)
                && Double.isFinite(imageHeight)
                && imageWidth >= 0.0
                && imageHeight >= 0.0
                && Double.isFinite(timestampSec);
        if (!finite) {
            return new PieceObservation(cameraId, 0.0, 0.0, 0.0, 0.0, 0.0, false);
        }
        return new PieceObservation(
                cameraId, imageX, imageY, imageWidth, imageHeight, timestampSec, true);
    }

    /** Recognition id from the camera. Not a stable physical track id. */
    public int getCameraId() {
        return cameraId;
    }

    public double getImageX() {
        return imageX;
    }

    public double getImageY() {
        return imageY;
    }

    public double getImageWidth() {
        return imageWidth;
    }

    public double getImageHeight() {
        return imageHeight;
    }

    public double getTimestampSec() {
        return timestampSec;
    }

    public boolean isValid() {
        return valid;
    }
}
