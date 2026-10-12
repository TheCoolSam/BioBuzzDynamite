package org.firstinspires.ftc.teamcode.scoring;

/** Measured grid in pitch radians and shoe gap mm. Coordinates are inches from yaw pivot.
 * Every grid interval must be validated, including under shooting load. */
public final class LauncherGeometry {
    public static final class Release {
        public final double forward,left,height,launchPitch;
        public Release(double forward,double left,double height,double launchPitch) {
            for(double n:new double[]{forward,left,height,launchPitch})if(!Double.isFinite(n))
                throw new IllegalArgumentException("Measured release geometry required");
            this.forward=forward;this.left=left;this.height=height;this.launchPitch=launchPitch;
        }
    }
    private final double[] pitches,gaps;
    private final Release[][] grid;
    public final CalibratedAxis pitchServo,gapServo;
    public final double errorInches;
    public LauncherGeometry(double[] pitches,double[] gaps,Release[][] grid,
            CalibratedAxis pitchServo,CalibratedAxis gapServo,double errorInches) {
        if(pitches==null||gaps==null||pitches.length<2||gaps.length<2||grid==null||grid.length!=pitches.length
                ||pitchServo==null||gapServo==null||!Double.isFinite(errorInches)||errorInches<0)
            throw new IllegalArgumentException("Measured pitch/gap release grid and error required");
        this.pitches=pitches.clone();this.gaps=gaps.clone();this.grid=new Release[pitches.length][gaps.length];
        this.pitchServo=pitchServo;this.gapServo=gapServo;this.errorInches=errorInches;
        for(int i=0;i<pitches.length;i++) {
            if(!Double.isFinite(pitches[i])||(i>0&&pitches[i]<=pitches[i-1])||!unit(pitchServo.map(pitches[i]))
                    ||grid[i]==null||grid[i].length!=gaps.length)throw new IllegalArgumentException("Pitch grid/mapping invalid");
            for(int j=0;j<gaps.length;j++) {
                if(!Double.isFinite(gaps[j])||gaps[j]<=0||(j>0&&gaps[j]<=gaps[j-1])
                        ||!unit(gapServo.map(gaps[j]))||grid[i][j]==null)throw new IllegalArgumentException("Gap grid/mapping invalid");
                this.grid[i][j]=grid[i][j];
            }
        }
    }
    private static boolean unit(double n) {return Double.isFinite(n)&&n>=0&&n<=1;}
    private static int interval(double[] knots,double value) {
        if(!Double.isFinite(value)||value<knots[0]||value>knots[knots.length-1])return -1;
        for(int i=1;i<knots.length;i++)if(value<=knots[i])return i-1;
        return -1;
    }
    public Release at(double pitch,double gap) {
        int i=interval(pitches,pitch),j=interval(gaps,gap);if(i<0||j<0)return null;
        double u=(pitch-pitches[i])/(pitches[i+1]-pitches[i]),v=(gap-gaps[j])/(gaps[j+1]-gaps[j]);
        Release a=grid[i][j],b=grid[i+1][j],c=grid[i][j+1],d=grid[i+1][j+1];
        return new Release(mix(a.forward,b.forward,c.forward,d.forward,u,v),mix(a.left,b.left,c.left,d.left,u,v),
                mix(a.height,b.height,c.height,d.height,u,v),mix(a.launchPitch,b.launchPitch,c.launchPitch,d.launchPitch,u,v));
    }
    private static double mix(double a,double b,double c,double d,double u,double v) {
        return (a+(b-a)*u)*(1-v)+(c+(d-c)*u)*v;
    }
}
