package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.match.AllianceColor;

/**
 * Driver stick mapping shared by every driver mode.
 *
 * <p>Turn: stick right is clockwise from above. Pedro and {@link BasicDrive} use
 * positive yaw as counterclockwise, so right-stick X is negated.
 *
 * <p>Field-centric translation: stick up is away from this alliance's driver wall.
 * Official BIOBUZZ/FTC +Y points from the Red wall toward Blue, so it cannot point
 * away from both driver stations. This repository uses Pedro's corner origin and
 * does not apply the 72 in / 90° FTC-center conversion documented in
 * {@link org.firstinspires.ftc.teamcode.pedro.PedroMappedReading}. Mapping official
 * FTC +Y through that conversion puts Red facing this frame's +X and Blue facing
 * −X. {@link AllianceColor#UNKNOWN} refuses field translation.
 *
 * <p>Robot-centric practice and drive recovery do not rotate for alliance: chassis
 * +X is right, +Y is forward.
 */
public final class DriverStickMap {
    public static final double DEADBAND=.08;
    /** Pedro heading a red driver faces when looking into the field. */
    public static final double RED_DRIVER_AWAY_HEADING_RAD=0;
    /** Pedro heading a blue driver faces when looking into the field. */
    public static final double BLUE_DRIVER_AWAY_HEADING_RAD=Math.PI;

    public static final class Command {
        public final double fieldX,fieldY,turn;
        public Command(double fieldX,double fieldY,double turn) {
            this.fieldX=fieldX;this.fieldY=fieldY;this.turn=turn;
        }
    }

    private DriverStickMap() { }

    public static double deadband(double value) {
        return Math.abs(value)<DEADBAND?0:value;
    }

    /** Stick right is clockwise from above. */
    public static double turn(double rightStickX) {
        return deadband(-rightStickX);
    }

    public static double driverAwayHeadingRad(AllianceColor alliance) {
        if(alliance==AllianceColor.RED)return RED_DRIVER_AWAY_HEADING_RAD;
        if(alliance==AllianceColor.BLUE)return BLUE_DRIVER_AWAY_HEADING_RAD;
        return Double.NaN;
    }

    /** Chassis +X right, +Y forward. Drive Practice and latched recovery. */
    public static Command robotCentric(double leftStickX,double leftStickY,double rightStickX) {
        return new Command(deadband(leftStickX),deadband(-leftStickY),turn(rightStickX));
    }

    /**
     * Stick up is away from the alliance wall. Unknown alliance zeroes translation
     * and still applies the clockwise turn mapping.
     */
    public static Command fieldCentric(double leftStickX,double leftStickY,double rightStickX,
            AllianceColor alliance) {
        double stickRight=deadband(leftStickX);
        double stickUp=deadband(-leftStickY);
        double heading=driverAwayHeadingRad(alliance);
        double yaw=turn(rightStickX);
        if(!Double.isFinite(heading))return new Command(0,0,yaw);
        double cos=Math.cos(heading),sin=Math.sin(heading);
        return new Command(stickUp*cos+stickRight*sin,stickUp*sin-stickRight*cos,yaw);
    }
}
