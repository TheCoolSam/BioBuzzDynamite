package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * Which pixel inside a detection is sent to the floor projection.
 *
 * <p>These pieces are spheres sitting on the floor. The bounding-box center is
 * the image of the ball's center, which is one radius above the floor contact.
 * On a camera that looks forward and down, that contact sits toward the bottom
 * of the silhouette. {@link #LOWER_CENTER} uses that point. It is still an
 * approximation: the true contact depends on the lens and the ball radius,
 * which is what the homography is calibrated to absorb.
 *
 * <p>HuskyLens Y increases downward, so the lower center is
 * {@code (x, y + height / 2)}.
 */
public enum ObservationAnchor {
    BOUNDING_BOX_CENTER,
    LOWER_CENTER
}
