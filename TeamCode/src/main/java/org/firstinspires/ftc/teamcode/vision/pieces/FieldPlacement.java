package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * Robot-floor inches to field inches.
 *
 * <p>Heading 0 points along field +X and positive heading is counterclockwise,
 * matching {@link org.firstinspires.ftc.teamcode.math.AngleUtil}. With
 * {@code xForward} along that heading and {@code yLeft} to the robot's left:
 *
 * <pre>
 * xField = robotX + cos(heading) * xForward - sin(heading) * yLeft
 * yField = robotY + sin(heading) * xForward + cos(heading) * yLeft
 * </pre>
 *
 * At heading 0, forward increases field X and left increases field Y.
 * At heading +π/2, forward increases field Y and left decreases field X.
 */
public final class FieldPlacement {

    private FieldPlacement() {
    }

    public static double fieldX(
            double robotX,
            double headingRad,
            double xForwardInches,
            double yLeftInches) {
        return robotX
                + Math.cos(headingRad) * xForwardInches
                - Math.sin(headingRad) * yLeftInches;
    }

    public static double fieldY(
            double robotY,
            double headingRad,
            double xForwardInches,
            double yLeftInches) {
        return robotY
                + Math.sin(headingRad) * xForwardInches
                + Math.cos(headingRad) * yLeftInches;
    }
}
