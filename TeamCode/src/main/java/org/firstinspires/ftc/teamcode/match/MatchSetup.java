package org.firstinspires.ftc.teamcode.match;

/**
 * The alliance picked on the gamepad during INIT. It lives as long as the robot app does, so the
 * choice made before autonomous carries into TeleOp without picking again.
 *
 * <p>Picking is the same in every competition OpMode: during INIT press X (Square) for BLUE or
 * B (Circle) for RED on gamepad 1. Nothing moves during INIT.
 */
public final class MatchSetup {
    private static AllianceColor alliance = AllianceColor.UNKNOWN;
    private static long pickedAtMillis;

    private MatchSetup() { }

    public static synchronized AllianceColor alliance() { return alliance; }

    public static synchronized boolean picked() { return alliance != AllianceColor.UNKNOWN; }

    public static synchronized void setAlliance(AllianceColor color) {
        if (color == null || color == AllianceColor.UNKNOWN) return;
        alliance = color;
        pickedAtMillis = System.currentTimeMillis();
    }

    /** Minutes since the alliance was picked, so stale choices from an earlier match stand out. */
    public static synchronized double minutesSincePick() {
        return picked() ? (System.currentTimeMillis() - pickedAtMillis) / 60000.0 : Double.NaN;
    }

    /**
     * Reads the INIT picker buttons. Returns true when the alliance changed.
     * Call only from init_loop: B is the emergency stop once the match is running.
     */
    public static boolean pickFromGamepad(boolean xBlue, boolean bRed) {
        if (xBlue && !bRed && alliance() != AllianceColor.BLUE) { setAlliance(AllianceColor.BLUE); return true; }
        if (bRed && !xBlue && alliance() != AllianceColor.RED) { setAlliance(AllianceColor.RED); return true; }
        return false;
    }
}
