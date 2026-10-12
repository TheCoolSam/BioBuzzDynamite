package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.*;
import org.firstinspires.ftc.teamcode.scoring.HiveTracker;

/** Chosen impact point in observed tag coordinates (inches). Calibrate each tag/hive state.
 * Lip clearance must include ball radius and measured spread, not just the center point. */
public final class CellTargetGeometry {
    public static final class Point {
        public final int tagId;
        public final HiveTracker.State state;
        public final double x,y,z,clearanceInches;
        public Point(int tagId,HiveTracker.State state,double x,double y,double z,double clearanceInches) {
            if(tagId<0||(state!=HiveTracker.State.SIDE_A_UP&&state!=HiveTracker.State.SIDE_B_UP)
                    ||!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(z)
                    ||!Double.isFinite(clearanceInches)||clearanceInches<=0)
                throw new IllegalArgumentException("Measured internal impact point and lip clearance required");
            this.tagId=tagId;this.state=state;this.x=x;this.y=y;this.z=z;this.clearanceInches=clearanceInches;
        }
    }
    private final List<Point> points;
    public CellTargetGeometry(List<Point> points) {
        if(points==null||points.isEmpty())throw new IllegalArgumentException("Tag-to-cell measurements required");
        this.points=new ArrayList<Point>(points);
        Set<String> keys=new HashSet<String>();
        for(Point p:this.points)if(p==null||!keys.add(p.tagId+":"+p.state))throw new IllegalArgumentException("Unique tag/state points required");
    }
    public Point point(int id,HiveTracker.State state) {
        for(Point p:points)if(p.tagId==id&&p.state==state)return p;
        return null;
    }
    /** Rz(yaw) Ry(pitch) Rx(roll). Limelight camera-space arrays must be remapped first; see LimelightCameraPose. */
    public static double[] rotation(double yaw,double pitch,double roll) {
        double cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),cr=Math.cos(roll),sr=Math.sin(roll);
        return new double[]{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
                sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};
    }
    public double[] offset(int id,HiveTracker.State state,double[] tagToCamera) {
        Point p=point(id,state);if(p==null||tagToCamera==null||tagToCamera.length!=9)return null;
        double[] v=new double[3];
        for(int i=0;i<3;i++) {
            v[i]=tagToCamera[3*i]*p.x+tagToCamera[3*i+1]*p.y+tagToCamera[3*i+2]*p.z;
            if(!Double.isFinite(v[i]))return null;
        }
        return v;
    }
}
