package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Builds the step list for the Simple Auto. Pure logic, no SDK, so it can be tested on a laptop.
 *
 * <p>Distances are robot-centric inches (positive Y = the way the robot drives forward, positive X =
 * strafe right) and are turned into drive TIME using speeds measured on the real robot. This needs no
 * odometry. It is also less accurate than odometry, so keep the plan simple until Pedro is tuned.
 *
 * <p>Routes:
 * <ul>
 *   <li>0 LEAVE: drive the PARK legs only (leave the starting zone).</li>
 *   <li>1 PRELOAD: optional move to a shot spot, fire the preload, then PARK legs.</li>
 *   <li>2 PRELOAD + PICKUP: as 1, then drive out with the intake on, drive back, fire again, then PARK legs.</li>
 * </ul>
 * Blue mirrors every strafe distance, so one set of Red numbers drives both alliances.
 */
public final class SimpleAutoPlan {
    private SimpleAutoPlan() { }

    public static final int ROUTE_LEAVE = 0;
    public static final int ROUTE_PRELOAD = 1;
    public static final int ROUTE_PRELOAD_PICKUP = 2;
    public static final double MAX_LEG_INCHES = 144.0;

    public enum Kind { WAIT, DRIVE, SHOOT }

    public static final class Step {
        public final Kind kind;
        public final String label;
        /** Drive command, -1..1 of the configured drive power. x right, y forward, yaw counterclockwise. */
        public final double x, y, yaw;
        public final double seconds;
        public final boolean intake;
        /** Final parking legs: the runner jumps here when time is running out. */
        public final boolean park;
        public final int shots;

        Step(Kind kind, String label, double x, double y, double yaw, double seconds, boolean intake, boolean park, int shots) {
            this.kind = kind; this.label = label; this.x = x; this.y = y; this.yaw = yaw;
            this.seconds = seconds; this.intake = intake; this.park = park; this.shots = shots;
        }
    }

    public static final class Config {
        public int route = ROUTE_PRELOAD;
        public boolean blue = false;
        public double startDelaySec = 0.0;
        public int shots = 3;
        /** Measured: inches travelled per second at the configured drive power. */
        public double inPerSecForward = 30.0;
        public double inPerSecStrafe = 24.0;
        public double shotMoveForwardIn = 0.0;
        public double shotMoveStrafeIn = 0.0;
        public double pickupForwardIn = 24.0;
        public double pickupStrafeIn = 0.0;
        public double pickupSettleSec = 0.3;
        public double parkForwardIn = 30.0;
        public double parkStrafeIn = 0.0;
    }

    /** Empty list means the config is usable. */
    public static List<String> validate(Config c) {
        List<String> problems = new ArrayList<String>();
        if (c == null) { problems.add("no config"); return problems; }
        if (c.route < ROUTE_LEAVE || c.route > ROUTE_PRELOAD_PICKUP) problems.add("ROUTE must be 0, 1 or 2");
        if (!(c.shots >= 1 && c.shots <= 4)) problems.add("SHOTS must be 1 to 4 (robot holds at most 4)");
        if (!finite(c.inPerSecForward) || c.inPerSecForward <= 0) problems.add("IN_PER_SEC_FORWARD must be > 0 (measure it)");
        if (!finite(c.inPerSecStrafe) || c.inPerSecStrafe <= 0) problems.add("IN_PER_SEC_STRAFE must be > 0 (measure it)");
        if (!(finite(c.startDelaySec) && c.startDelaySec >= 0 && c.startDelaySec <= 15)) problems.add("START_DELAY must be 0 to 15 s");
        if (!(finite(c.pickupSettleSec) && c.pickupSettleSec >= 0 && c.pickupSettleSec <= 3)) problems.add("PICKUP_SETTLE must be 0 to 3 s");
        double[] legs = {c.shotMoveForwardIn, c.shotMoveStrafeIn, c.pickupForwardIn, c.pickupStrafeIn, c.parkForwardIn, c.parkStrafeIn};
        for (double leg : legs) if (!finite(leg) || Math.abs(leg) > MAX_LEG_INCHES) { problems.add("a leg distance is NaN or beyond " + (int) MAX_LEG_INCHES + " in"); break; }
        if (c.route == ROUTE_LEAVE && c.parkForwardIn == 0 && c.parkStrafeIn == 0) problems.add("LEAVE route has nothing to drive: set PARK_FORWARD_IN or PARK_STRAFE_IN");
        if (c.route == ROUTE_PRELOAD_PICKUP && c.pickupForwardIn == 0 && c.pickupStrafeIn == 0) problems.add("PICKUP route has nothing to drive: set PICKUP_FORWARD_IN or PICKUP_STRAFE_IN");
        return problems;
    }

    /** Builds the steps. Throws IllegalArgumentException if {@link #validate} reports problems. */
    public static List<Step> build(Config c) {
        List<String> problems = validate(c);
        if (!problems.isEmpty()) throw new IllegalArgumentException(problems.get(0));
        double mirror = c.blue ? -1.0 : 1.0;
        List<Step> steps = new ArrayList<Step>();
        if (c.startDelaySec > 0) steps.add(new Step(Kind.WAIT, "START DELAY", 0, 0, 0, c.startDelaySec, false, false, 0));

        if (c.route >= ROUTE_PRELOAD) {
            legs(steps, c, "SHOT MOVE", c.shotMoveForwardIn, c.shotMoveStrafeIn * mirror, false, false);
            steps.add(new Step(Kind.SHOOT, "SHOOT PRELOAD", 0, 0, 0, 0, false, false, c.shots));
        }
        if (c.route == ROUTE_PRELOAD_PICKUP) {
            double sx = c.pickupStrafeIn * mirror;
            // Out: forward then strafe. Back: undo in the opposite order so the robot retraces its path.
            legs(steps, c, "PICKUP OUT", c.pickupForwardIn, sx, true, false);
            if (c.pickupSettleSec > 0) steps.add(new Step(Kind.WAIT, "PICKUP SETTLE", 0, 0, 0, c.pickupSettleSec, true, false, 0));
            addLeg(steps, c, "PICKUP BACK (strafe)", -sx, 0, true, false);
            addLeg(steps, c, "PICKUP BACK (forward)", 0, -c.pickupForwardIn, true, false);
            steps.add(new Step(Kind.SHOOT, "SHOOT SECOND", 0, 0, 0, 0, false, false, c.shots));
        }
        legs(steps, c, c.route == ROUTE_LEAVE ? "LEAVE" : "PARK", c.parkForwardIn, c.parkStrafeIn * mirror, false, true);
        return Collections.unmodifiableList(steps);
    }

    private static void legs(List<Step> steps, Config c, String label, double forwardIn, double strafeIn, boolean intake, boolean park) {
        addLeg(steps, c, label + " (forward)", 0, forwardIn, intake, park);
        addLeg(steps, c, label + " (strafe)", strafeIn, 0, intake, park);
    }

    private static void addLeg(List<Step> steps, Config c, String label, double strafeIn, double forwardIn, boolean intake, boolean park) {
        if (strafeIn == 0 && forwardIn == 0) return;
        double seconds = strafeIn != 0 ? Math.abs(strafeIn) / c.inPerSecStrafe : Math.abs(forwardIn) / c.inPerSecForward;
        steps.add(new Step(Kind.DRIVE, label, Math.signum(strafeIn), Math.signum(forwardIn), 0, seconds, intake, park, 0));
    }

    /**
     * Longest the plan can take if every shot waits out all of its timeouts. Compare with the 30 s
     * autonomous period. Uses the same limits as {@link SimpleAutoRunner.Params}.
     */
    public static double worstCaseSeconds(List<Step> steps, SimpleAutoRunner.Params p) {
        double total = 0;
        for (Step s : steps) {
            if (s.kind == Kind.SHOOT) {
                total += p.spinupMaxSec + s.shots * (p.feedPulseSec + p.recoverMaxSec) + p.finalSettleSec;
            } else {
                total += s.seconds + driveRampSeconds(s, p);
            }
        }
        return total;
    }

    /** Ramping up from zero costs half of the ramp time in full-speed distance, so the leg takes that much longer. */
    private static double driveRampSeconds(Step s, SimpleAutoRunner.Params p) {
        return s.kind == Kind.DRIVE ? 0.5 / p.slewPerSec : 0;
    }

    /** Time the plan takes if every shot is ready right away. */
    public static double bestCaseSeconds(List<Step> steps, SimpleAutoRunner.Params p) {
        double total = 0;
        for (Step s : steps) {
            if (s.kind == Kind.SHOOT) {
                total += p.readyHoldSec + s.shots * p.feedPulseSec + Math.max(0, s.shots - 1) * p.minRecoverSec + p.finalSettleSec;
            } else {
                total += s.seconds + driveRampSeconds(s, p);
            }
        }
        return total;
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
