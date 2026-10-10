package org.firstinspires.ftc.teamcode.robot;

/** Bounded timing samples, exported periodically rather than sorted in every control loop. */
public final class LoopMetrics {
    private final double[] seconds=new double[256];
    private int count,cursor;
    private double max;
    public void record(double elapsedSec) {
        if (!Double.isFinite(elapsedSec)||elapsedSec<0) return;
        seconds[cursor++%seconds.length]=elapsedSec;count=Math.min(count+1,seconds.length);max=Math.max(max,elapsedSec);
    }
    public double[] summary() {
        if (count==0) return new double[]{0,0,0,0};
        double[] copy=java.util.Arrays.copyOf(seconds,count);java.util.Arrays.sort(copy);
        return new double[]{copy[(int)((count-1)*.5)],copy[(int)((count-1)*.95)],copy[(int)((count-1)*.99)],max};
    }
}
