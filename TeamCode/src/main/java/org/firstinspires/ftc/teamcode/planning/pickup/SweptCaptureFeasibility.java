package org.firstinspires.ftc.teamcode.planning.pickup;

import org.firstinspires.ftc.teamcode.math.AngleUtil;

/** Conservative rotation-independent footprint and two-segment intake approach gate.
 * Radius must enclose the measured robot/intake/turret envelope in all enabled poses.
 * Expanded obstacle rectangles intentionally reject some paths a detailed planner could use.
 */
public final class SweptCaptureFeasibility implements CaptureFeasibilityModel {
    private final double width, height, radius;
    private final double halfForward, halfLeft;
    private final double[][] obstacles;
    public SweptCaptureFeasibility(double width, double height, double radius, double[][] obstacles) {
        this(width,height,radius,obstacles,Double.NaN,Double.NaN);
    }
    public SweptCaptureFeasibility(double width, double height, double radius, double[][] obstacles,
            double halfForward, double halfLeft) {
        if (!Double.isFinite(width) || !Double.isFinite(height) || !Double.isFinite(radius)
                || radius <= 0 || width <= 2*radius || height <= 2*radius) {
            throw new IllegalArgumentException("Measured enclosing radius and field dimensions required");
        }
        if(Double.isFinite(halfForward)||Double.isFinite(halfLeft)) {
            if(!Double.isFinite(halfForward)||!Double.isFinite(halfLeft)||halfForward<=0||halfLeft<=0
                    ||Math.hypot(halfForward,halfLeft)>radius+1e-8)
                throw new IllegalArgumentException("Oriented footprint must fit the enclosing radius");
        }
        this.width=width; this.height=height; this.radius=radius;
        this.halfForward=halfForward; this.halfLeft=halfLeft;
        this.obstacles = new double[obstacles == null ? 0 : obstacles.length][4];
        for (int i=0;i<this.obstacles.length;i++) {
            if (obstacles[i] == null || obstacles[i].length != 4) throw new IllegalArgumentException("Rectangle required");
            for (int j=0;j<4;j++) {
                if (!Double.isFinite(obstacles[i][j])) throw new IllegalArgumentException("Finite obstacle required");
                this.obstacles[i][j]=obstacles[i][j];
            }
            if (obstacles[i][0]>obstacles[i][2] || obstacles[i][1]>obstacles[i][3]) throw new IllegalArgumentException("Ordered rectangle required");
        }
    }
    public boolean segmentSafe(double x0,double y0,double x1,double y1) {
        if (!inside(x0,y0) || !inside(x1,y1)) return false;
        for (double[] box : obstacles) {
            double[] range={0,1};
            if (slab(x0,x1-x0,box[0]-radius,box[2]+radius,range)
                    && slab(y0,y1-y0,box[1]-radius,box[3]+radius,range)) return false;
        }
        return true;
    }
    /** Fixed-heading departure only: measured rectangle at the walls, full circle at obstacles.
     * The caller must suppress yaw and recheck the actual heading on every update. */
    public boolean departureSafe(double x0,double y0,double x1,double y1,double heading,
            double halfForward,double halfLeft) {
        if(!Double.isFinite(heading)||!Double.isFinite(halfForward)||!Double.isFinite(halfLeft)
                ||halfForward<=0||halfLeft<=0||Math.hypot(halfForward,halfLeft)>radius+1e-8)return false;
        double hx=Math.abs(Math.cos(heading))*halfForward+Math.abs(Math.sin(heading))*halfLeft;
        double hy=Math.abs(Math.sin(heading))*halfForward+Math.abs(Math.cos(heading))*halfLeft;
        for(double x:new double[]{x0,x1})if(!Double.isFinite(x)||x<hx-1e-8||x>width-hx+1e-8)return false;
        for(double y:new double[]{y0,y1})if(!Double.isFinite(y)||y<hy-1e-8||y>height-hy+1e-8)return false;
        for(double[] box:obstacles) {
            double[] range={0,1};
            if(slab(x0,x1-x0,box[0]-radius,box[2]+radius,range)
                    &&slab(y0,y1-y0,box[1]-radius,box[3]+radius,range))return false;
        }
        return true;
    }
    public double halfForward() {return halfForward;}
    public double halfLeft() {return halfLeft;}
    public boolean hasOrientedFootprint() {
        return Double.isFinite(halfForward)&&Double.isFinite(halfLeft)&&halfForward>0&&halfLeft>0;
    }
    /** Oriented rectangle versus walls; enclosing circle versus obstacles. */
    public boolean poseSafe(double x,double y,double heading,double halfForward,double halfLeft) {
        return departureSafe(x,y,x,y,heading,halfForward,halfLeft);
    }
    /**
     * Wall-adjacent park/waypoint legs: sample the oriented footprint while heading
     * interpolates. Falls back to the enclosing circle when extents are unavailable.
     */
    public boolean pathSafe(double x0,double y0,double heading0,double x1,double y1,double heading1,
            double halfForward,double halfLeft) {
        if(!Double.isFinite(halfForward)||!Double.isFinite(halfLeft)||halfForward<=0||halfLeft<=0)
            return segmentSafe(x0,y0,x1,y1);
        if(!Double.isFinite(heading0)||!Double.isFinite(heading1))return false;
        if(!segmentObstaclesOnly(x0,y0,x1,y1))return false;
        double dist=Math.hypot(x1-x0,y1-y0);
        double dh=Math.abs(AngleUtil.wrapRadians(heading1-heading0));
        int n=Math.max(1,(int)Math.ceil(dist)+(int)Math.ceil(dh/(.09)));
        for(int i=0;i<=n;i++) {
            double t=i/(double)n;
            double x=x0+t*(x1-x0),y=y0+t*(y1-y0);
            double heading=heading0+t*AngleUtil.wrapRadians(heading1-heading0);
            if(!poseSafe(x,y,heading,halfForward,halfLeft))return false;
        }
        return true;
    }
    public boolean pathSafe(double x0,double y0,double heading0,double x1,double y1,double heading1) {
        return pathSafe(x0,y0,heading0,x1,y1,heading1,halfForward,halfLeft);
    }
    private boolean segmentObstaclesOnly(double x0,double y0,double x1,double y1) {
        for(double[] box:obstacles) {
            double[] range={0,1};
            if(slab(x0,x1-x0,box[0]-radius,box[2]+radius,range)
                    &&slab(y0,y1-y0,box[1]-radius,box[3]+radius,range))return false;
        }
        return true;
    }
    private boolean inside(double x,double y) {
        return Double.isFinite(x) && Double.isFinite(y) && x>=radius && x<=width-radius
                && y>=radius && y<=height-radius;
    }
    private static boolean slab(double start,double delta,double low,double high,double[] range) {
        if (Math.abs(delta)<1e-12) return start>=low && start<=high;
        double a=(low-start)/delta, b=(high-start)/delta;
        range[0]=Math.max(range[0],Math.min(a,b)); range[1]=Math.min(range[1],Math.max(a,b));
        return range[0]<=range[1];
    }
    @Override public boolean isFeasible(double x,double y,double heading,PickupTarget target) {
        return target != null && target.getPiece() != null && Double.isFinite(heading)
                && Double.isFinite(target.getApproachHeadingRad())
                && segmentSafe(x,y,target.getApproachX(),target.getApproachY())
                && segmentSafe(target.getApproachX(),target.getApproachY(),target.getCaptureX(),target.getCaptureY());
    }
}
