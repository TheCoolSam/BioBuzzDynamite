package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Seconds from a pose to a completed tip shot.
 *
 * <p>The pose is either the robot now, for shoot-now, or the capture pose
 * after the last pickup. The result should include travel to a scoring pose,
 * rotation onto the shot heading, and the shot itself. The planner does not
 * know what the shooter or the path follower is. Replace this estimate later
 * without changing the search.
 */
public interface ShotSetupTimeModel {

    double estimateSeconds(double robotX, double robotY, double robotHeading, BallLoad load);
}
