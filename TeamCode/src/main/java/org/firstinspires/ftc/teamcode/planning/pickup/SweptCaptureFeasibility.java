package org.firstinspires.ftc.teamcode.planning.pickup;

/** Conservative rotation-independent footprint and two-segment intake approach gate.
 * Radius must enclose the measured robot/intake/turret envelope in all enabled poses.
 * Expanded obstacle rectangles intentionally reject some paths a detailed planner could use.
 */
public final class SweptCaptureFeasibility implements CaptureFeasibilityModel {
    private final double width, height, radius;
    private final double[][] obstacles;
    public SweptCaptureFeasibility(double width, double height, double radius, double[][] obstacles) {
        if (!Double.isFinite(width) || !Double.isFinite(height) || !Double.isFinite(radius)
                || radius <= 0 || width <= 2*radius || height <= 2*radius) {
            throw new IllegalArgumentException("Measured enclosing radius and field dimensions required");
        }
        this.width=width; this.height=height; this.radius=radius;
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
