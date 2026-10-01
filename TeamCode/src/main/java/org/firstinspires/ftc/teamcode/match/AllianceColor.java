package org.firstinspires.ftc.teamcode.match;

/**
 * Which alliance the robot is playing. Vision uses this to label nectar.
 * The pickup planner does not import it and does not learn red versus blue.
 */
public enum AllianceColor {
    RED,
    BLUE,
    UNKNOWN
}
