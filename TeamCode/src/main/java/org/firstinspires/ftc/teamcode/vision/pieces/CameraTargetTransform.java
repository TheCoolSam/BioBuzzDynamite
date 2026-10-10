package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.state.*;
import org.firstinspires.ftc.teamcode.scoring.HiveTracker;

/** Explicit measured optical-axis rotation, camera translation, and yaw-pivot offsets. */
public final class CameraTargetTransform {
    private final double[] rotation;
    private final double pivotForward,pivotLeft,cameraForward,cameraLeft,minRange,maxRange;
    public double pivotForward() {return pivotForward;}
    public double pivotLeft() {return pivotLeft;}
    public CameraTargetTransform(double[] opticalToTurret,double pivotForward,double pivotLeft,
            double cameraForward,double cameraLeft,double minRange,double maxRange) {
        if(opticalToTurret==null||opticalToTurret.length!=9)throw new IllegalArgumentException("Measured 3x3 rotation required");
        rotation=opticalToTurret.clone();
        for(double n:rotation)if(!Double.isFinite(n))throw new IllegalArgumentException("Finite rotation required");
        for(int a=0;a<3;a++)for(int b=0;b<3;b++) {
            double dot=0;for(int j=0;j<3;j++)dot+=rotation[3*a+j]*rotation[3*b+j];
            if(Math.abs(dot-(a==b?1:0))>1e-6)throw new IllegalArgumentException("Orthonormal rotation required");
        }
        double det=rotation[0]*(rotation[4]*rotation[8]-rotation[5]*rotation[7])
                -rotation[1]*(rotation[3]*rotation[8]-rotation[5]*rotation[6])
                +rotation[2]*(rotation[3]*rotation[7]-rotation[4]*rotation[6]);
        if(Math.abs(det-1)>1e-6)throw new IllegalArgumentException("Right-handed rotation required");
        for(double n:new double[]{pivotForward,pivotLeft,cameraForward,cameraLeft,minRange,maxRange})if(!Double.isFinite(n))throw new IllegalArgumentException("Measured offsets required");
        if(minRange<=0||maxRange<=minRange)throw new IllegalArgumentException("Measured useful range required");
        this.pivotForward=pivotForward;this.pivotLeft=pivotLeft;this.cameraForward=cameraForward;this.cameraLeft=cameraLeft;this.minRange=minRange;this.maxRange=maxRange;
    }
    public TargetEstimate project(double cameraX,double cameraY,double cameraZ,double exposureSec,
            int id,HiveTracker.State hive,RobotStateHistory history,TurretPoseHistory turret,long generation) {
        if(history==null||turret==null)return null;
        double forward=rotation[0]*cameraX+rotation[1]*cameraY+rotation[2]*cameraZ;
        double left=rotation[3]*cameraX+rotation[4]*cameraY+rotation[5]*cameraZ;
        double distance=Math.sqrt(cameraX*cameraX+cameraY*cameraY+cameraZ*cameraZ);
        if(!Double.isFinite(distance)||distance<minRange||distance>maxRange||forward<=0)return null;
        java.util.Optional<RobotState> at=history.sampleAt(exposureSec);
        double angle=turret.at(exposureSec,generation);
        if(!at.isPresent()||!Double.isFinite(angle)||at.get().getResetGeneration()!=generation)return null;
        RobotState robot=at.get();
        if(!robot.isFresh(exposureSec,.2))return null;
        double fx=cameraForward+forward,fy=cameraLeft+left;
        double chassisX=pivotForward+Math.cos(angle)*fx-Math.sin(angle)*fy;
        double chassisY=pivotLeft+Math.sin(angle)*fx+Math.cos(angle)*fy;
        return new TargetEstimate(robot.getFieldX()+Math.cos(robot.getHeadingRad())*chassisX-Math.sin(robot.getHeadingRad())*chassisY,
                robot.getFieldY()+Math.sin(robot.getHeadingRad())*chassisX+Math.cos(robot.getHeadingRad())*chassisY,
                exposureSec,id,generation,hive);
    }
}
