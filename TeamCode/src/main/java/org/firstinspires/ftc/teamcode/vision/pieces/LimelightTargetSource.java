package org.firstinspires.ftc.teamcode.vision.pieces;

import com.qualcomm.hardware.limelightvision.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.state.RobotStateHistory;
import org.firstinspires.ftc.teamcode.scoring.HiveTracker;

/** SDK adapter using relative camera geometry and exposure histories. Never injects botpose. */
public final class LimelightTargetSource {
    public interface CellClassifier { HiveTracker.State classify(LLResultTypes.FiducialResult target); }
    private final Limelight3A camera;
    private final CameraTargetTransform transform;
    private final int pipeline;
    private final int[] allowedIds;
    private final CellClassifier classifier;
    private double lastTimestamp=Double.NEGATIVE_INFINITY;
    private TargetEstimate cached;
    public LimelightTargetSource(Limelight3A camera,CameraTargetTransform transform,int pipeline,
            int[] allianceCellIds,CellClassifier classifier) {
        if(camera==null||transform==null||allianceCellIds==null||allianceCellIds.length==0||classifier==null||pipeline<0)throw new IllegalArgumentException("Configured target identity/geometry/classifier required");
        this.camera=camera;this.transform=transform;this.pipeline=pipeline;allowedIds=allianceCellIds.clone();this.classifier=classifier;
    }
    public void start() { camera.pipelineSwitch(pipeline);camera.start(); }
    public void stop() { camera.stop();reset(); }
    public void reset() { lastTimestamp=Double.NEGATIVE_INFINITY;cached=null; }
    public TargetEstimate read(double now,RobotStateHistory history,TurretPoseHistory turret,long generation) {
        LLResult result=camera.getLatestResult();
        if(result==null||!result.isValid()||result.getPipelineIndex()!=pipeline||result.getStaleness()<0||result.getStaleness()>200) {cached=null;return null;}
        double timestamp=result.getTimestamp();
        if(!Double.isFinite(timestamp)||timestamp<lastTimestamp) {reset();return null;}
        if(timestamp==lastTimestamp)return cached!=null&&cached.fresh(now,.2,generation)?cached:null;
        lastTimestamp=timestamp;cached=null;
        double latency=result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency()+result.getParseLatency();
        if(!Double.isFinite(latency)||latency<0||latency>200)return null;
        double exposure=now-latency*.001;
        for(LLResultTypes.FiducialResult target:result.getFiducialResults()) {
            boolean allowed=false;for(int id:allowedIds)if(id==target.getFiducialId())allowed=true;
            if(!allowed||target.getTargetPoseCameraSpace()==null)continue;
            Position position=target.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            cached=transform.project(position.x,position.y,position.z,exposure,target.getFiducialId(),classifier.classify(target),history,turret,generation);
            if(cached!=null)return cached;
        }
        return null;
    }
}
