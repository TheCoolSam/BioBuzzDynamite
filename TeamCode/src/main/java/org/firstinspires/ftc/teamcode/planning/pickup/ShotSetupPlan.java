package org.firstinspires.ftc.teamcode.planning.pickup;

/** The selected executable setup pose, with the same cost the planner used. */
public final class ShotSetupPlan {
    public final double x, y, headingRad, seconds;
    public ShotSetupPlan(double x,double y,double headingRad,double seconds) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingRad)
                || !Double.isFinite(seconds) || seconds<0) throw new IllegalArgumentException("Finite shot setup required");
        this.x=x; this.y=y; this.headingRad=headingRad; this.seconds=seconds;
    }
}
