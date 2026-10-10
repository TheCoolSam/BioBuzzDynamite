package org.firstinspires.ftc.teamcode.scoring;

/** Requires distinct stable observations; target changes never authorize an immediate release. */
public final class HiveTracker {
    public enum State { UNKNOWN, TIPPING, SIDE_A_UP, SIDE_B_UP }
    private State candidate=State.UNKNOWN, state=State.UNKNOWN;
    private double since=Double.NaN,lastSeen=Double.NaN;
    private final double settleSec,maxAgeSec;
    public HiveTracker(double settleSec,double maxAgeSec) {
        if (!Double.isFinite(settleSec)||settleSec<=0||!Double.isFinite(maxAgeSec)||maxAgeSec<=0) throw new IllegalArgumentException("Measured HIVE timing required");
        this.settleSec=settleSec;this.maxAgeSec=maxAgeSec;
    }
    public void observe(State observed,double acquiredSec,boolean valid) {
        if (!valid || observed==null || !Double.isFinite(acquiredSec) || observed==State.UNKNOWN) { reset();return; }
        if (Double.isFinite(lastSeen) && acquiredSec<=lastSeen) return;
        if (observed!=candidate || !Double.isFinite(lastSeen) || acquiredSec-lastSeen>maxAgeSec) {
            candidate=observed;since=acquiredSec;state=observed==State.TIPPING?observed:State.UNKNOWN;
        }
        lastSeen=acquiredSec;
        if (acquiredSec-since>=settleSec && observed!=State.TIPPING) state=observed;
    }
    public State state(double nowSec) {
        if (!Double.isFinite(nowSec)||!Double.isFinite(lastSeen)||nowSec<lastSeen||nowSec-lastSeen>maxAgeSec) return State.UNKNOWN;
        return state;
    }
    public void reset() { candidate=state=State.UNKNOWN;since=lastSeen=Double.NaN; }
}
