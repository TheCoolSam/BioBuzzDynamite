package org.firstinspires.ftc.teamcode.robot;

/** Robot-centric mecanum mix: x right, y forward, yaw counterclockwise. No localization. */
public final class BasicDrive {
    public static final class Settings {
        public final String[] names;
        public final boolean[] reversed;
        public final double powerCap;
        public Settings(String[] names,boolean[] reversed,double powerCap,boolean signsAndStopVerified) {
            if(!signsAndStopVerified||names==null||names.length!=4||reversed==null||reversed.length!=4
                    ||!Double.isFinite(powerCap)||powerCap<=0||powerCap>1)
                throw new IllegalArgumentException("Verified drive names, directions, stop and cap required");
            java.util.Set<String> unique=new java.util.HashSet<String>();
            for(String name:names)if(name==null||name.trim().isEmpty()||!unique.add(name))
                throw new IllegalArgumentException("Four unique drive names required");
            this.names=names.clone();this.reversed=reversed.clone();this.powerCap=powerCap;
        }
    }
    public static double[] powers(double x,double y,double yaw,double cap) {
        for(double n:new double[]{x,y,yaw})if(!Double.isFinite(n)||Math.abs(n)>1)return null;
        if(!Double.isFinite(cap)||cap<=0||cap>1)return null;
        double[] powers={y+x-yaw,y-x-yaw,y-x+yaw,y+x+yaw}; // FL, BL, FR, BR
        double scale=1;for(double p:powers)scale=Math.max(scale,Math.abs(p));
        for(int i=0;i<4;i++)powers[i]=powers[i]/scale*cap;
        return powers;
    }
    private BasicDrive() { }
}
