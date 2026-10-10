package org.firstinspires.ftc.teamcode.robot;

/** Both edges require an uninterrupted measured dwell; initialization preserves physical state. */
public final class BooleanDebouncer {
    private final double dwell;
    private boolean initialized,stable,candidate;
    private double since,lastNow;
    public BooleanDebouncer(double dwell) {
        if(!Double.isFinite(dwell)||dwell<=0)throw new IllegalArgumentException("Measured sensor dwell required");
        this.dwell=dwell;
    }
    public boolean update(boolean raw,double now) {
        if(!Double.isFinite(now)||(initialized&&now<lastNow))throw new IllegalStateException("Sensor clock invalid");
        lastNow=now;
        if(!initialized) {initialized=true;stable=candidate=raw;since=now;return stable;}
        if(raw!=candidate) {candidate=raw;since=now;}
        if(now-since>=dwell)stable=candidate;
        return stable;
    }
}
