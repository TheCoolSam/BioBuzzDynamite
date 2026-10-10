package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.*;

/** Acquisition-time output angles in the continuous legal travel interval. */
public final class TurretPoseHistory {
    private final List<double[]> samples=new ArrayList<double[]>();
    private long generation=Long.MIN_VALUE;
    public void clear() { samples.clear();generation=Long.MIN_VALUE; }
    public void add(double acquiredSec,double angle,long generation) {
        if (!Double.isFinite(acquiredSec)||!Double.isFinite(angle)) {clear();return;}
        if(this.generation!=generation) {clear();this.generation=generation;}
        if(!samples.isEmpty()&&acquiredSec<=samples.get(samples.size()-1)[0])return;
        samples.add(new double[]{acquiredSec,angle});
        while(samples.size()>200||acquiredSec-samples.get(0)[0]>1.5)samples.remove(0);
    }
    public double at(double exposureSec,long generation) {
        if(this.generation!=generation||samples.isEmpty()||!Double.isFinite(exposureSec))return Double.NaN;
        double[] previous=samples.get(0);
        if(Math.abs(exposureSec-previous[0])<=.002)return previous[1];
        if(exposureSec<previous[0])return Double.NaN;
        for(double[] next:samples) {
            if(Math.abs(exposureSec-next[0])<=.002)return next[1];
            if(next[0]>exposureSec) {
                if(next[0]-previous[0]>.2)return Double.NaN;
                return previous[1]+(next[1]-previous[1])*(exposureSec-previous[0])/(next[0]-previous[0]);
            }
            previous=next;
        }
        return Double.NaN;
    }
}
