package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * A point on the floor, in inches, relative to the robot.
 *
 * <p>{@code xForward} is along the robot heading. Heading 0 faces field +X,
 * so forward is field +X when the robot is facing that way.
 * {@code yLeft} is to the robot's left, which is field +Y at heading 0.
 * Both follow {@link org.firstinspires.ftc.teamcode.math.AngleUtil}.
 */
public final class FloorPoint {

    private final double xForwardInches;
    private final double yLeftInches;

    private FloorPoint(double xForwardInches, double yLeftInches) {
        this.xForwardInches = xForwardInches;
        this.yLeftInches = yLeftInches;
    }

    public static FloorPoint of(double xForwardInches, double yLeftInches) {
        if (!Double.isFinite(xForwardInches) || !Double.isFinite(yLeftInches)) {
            return null;
        }
        return new FloorPoint(xForwardInches, yLeftInches);
    }

    public double getXForwardInches() {
        return xForwardInches;
    }

    public double getYLeftInches() {
        return yLeftInches;
    }
}
