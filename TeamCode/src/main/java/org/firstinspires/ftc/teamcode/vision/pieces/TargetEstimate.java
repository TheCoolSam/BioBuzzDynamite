package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.scoring.HiveTracker;

/** Relative camera observation transformed into the project frame, never a localization correction. */
public final class TargetEstimate {
    public final double x,y,exposureSec;
    public final int targetId;
    public final long generation;
    public final HiveTracker.State hive;
    public TargetEstimate(double x,double y,double exposureSec,int targetId,long generation,HiveTracker.State hive) {
        this.x=x;this.y=y;this.exposureSec=exposureSec;this.targetId=targetId;this.generation=generation;
        this.hive=hive==null?HiveTracker.State.UNKNOWN:hive;
    }
    public boolean fresh(double now,double maxAge,long generation) {
        return this.generation==generation && Double.isFinite(x)&&Double.isFinite(y)&&Double.isFinite(exposureSec)
                && Double.isFinite(now)&&now>=exposureSec&&now-exposureSec<=maxAge;
    }
}
