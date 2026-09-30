package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Same shot time from every pose. Useful when a test is about pickup order
 * and does not want the route endpoint to change the ranking.
 * A match should pass a model that actually depends on where the route ends.
 */
public final class FixedShotSetupModel implements ShotSetupTimeModel {

    private final double seconds;

    public FixedShotSetupModel(double seconds) {
        this.seconds = Double.isFinite(seconds) && seconds > 0.0 ? seconds : 0.0;
    }

    @Override
    public double estimateSeconds(double robotX, double robotY, double robotHeading, BallLoad load) {
        return seconds;
    }
}
