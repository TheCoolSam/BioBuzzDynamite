package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.robot.RobotIO;

/** Bounded shot telemetry, including state transitions and 20 Hz measured behavior.
 * Outcome is deliberately unknown until an observer supplies it using the shot ID. */
public final class ShotJournal {
    public static final String HEADER="revision,shotId,time,event,type,robotX,robotY,vx,vy,omega,ax,ay,alpha,targetId,targetX,targetY,targetZ,yawCommand,yawMeasured,yawRate,pitchCommand,pitchMeasured,pitchRate,gapCommand,gapMeasured,gapRate,rpmCommand,rpmMeasured,rpmMinimum,feeder,staged,exit,release,flight,arrivalMin,arrivalMax,missBound,reason,feederStartSec,exitSec,rpmRecoverySec,outcomeScore,outcomeTip,runId";
    private final Deque<String> records=new ArrayDeque<String>();
    private final String revision;
    private final String runId;
    private long lastConfirmedShotId;
    private long id;
    private double lastSample=Double.NEGATIVE_INFINITY,minimumRpm=Double.NaN;
    private double feederStarted=Double.NaN,exitedAt=Double.NaN,recoverySince=Double.NaN,recoverySec=Double.NaN;
    private ScoringController.State previous=ScoringController.State.IDLE;
    private boolean previousExit;
    public long dropped;
    public ShotJournal(String revision) {
        this(revision,"UNSPECIFIED");
    }
    public ShotJournal(String revision,String runId) {
        if(revision==null||revision.trim().isEmpty())throw new IllegalArgumentException("Calibration revision required");
        this.revision=revision;
        if(runId==null||runId.trim().isEmpty())throw new IllegalArgumentException("Run ID required");
        this.runId=runId;
    }
    private static String cell(Object value) {return value==null?"":"\""+value.toString().replace("\"","\"\"")+"\"";}
    private void add(String record) {if(records.size()==512) {records.removeFirst();dropped++;}records.addLast(record);}
    public String poll() {return records.pollFirst();}
    public long currentShotId() {return id;}
    public long lastConfirmedShotId() {return lastConfirmedShotId;}
    public void outcome(long shotId,double now,String score,String tip) {
        if(shotId<1||shotId>id||!Double.isFinite(now))throw new IllegalArgumentException("Known shot ID and observation time required");
        Object[] values=new Object[44];values[0]=revision;values[1]=shotId;values[2]=now;values[3]="OUTCOME";values[43]=runId;
        values[41]=score;values[42]=tip;add(row(values));
    }
    public void record(double now,ScoringController.State state,RobotIO.Snapshot s,RobotIO.Command c,
            ShotRecipe recipe,MotionShotSolver.Solution solution,double yawCommand,double ax,double ay,double alpha,double miss) {
        boolean feederStart=state==ScoringController.State.FEED_ONE&&previous!=state;
        boolean exited=s.exit&&!previousExit;
        if(exited&&state==ScoringController.State.WAIT_FOR_HIVE
                &&(previous==ScoringController.State.FEED_ONE||previous==ScoringController.State.CONFIRM_EXIT))lastConfirmedShotId=id;
        if(feederStart) {id++;minimumRpm=s.rpm;feederStarted=now;exitedAt=recoverySince=recoverySec=Double.NaN;}
        if(exited&&id>0)exitedAt=now;
        if(Double.isFinite(exitedAt)&&recipe!=null&&c.rpm>0) {
            if(Double.isFinite(s.rpm)&&Math.abs(s.rpm-recipe.rpm)<=recipe.rpmTolerance) {
                if(!Double.isFinite(recoverySince))recoverySince=s.acquiredSec;
                if(!Double.isFinite(recoverySec)&&s.acquiredSec-recoverySince>=recipe.rpmStableSec)
                    recoverySec=s.acquiredSec-exitedAt;
            }else recoverySince=Double.NaN;
        }
        if(id>0&&state!=ScoringController.State.IDLE&&c.rpm>0&&Double.isFinite(s.rpm))minimumRpm=Double.isFinite(minimumRpm)?Math.min(minimumRpm,s.rpm):s.rpm;
        boolean changed=state!=previous||exited;
        previous=state;previousExit=s.exit;
        if(!changed&&(state==ScoringController.State.IDLE&&c.rpm==0||now-lastSample<.05))return;
        lastSample=now;
        String event=feederStart?"FEED_START":exited?"EXIT":changed?state.toString():"SAMPLE";
        LauncherFeedback f=s.launcher;
        ShotSelector.Selection selected=solution==null?null:solution.selection;
        double release=solution==null?Double.NaN:solution.releaseSec;
        double flight=selected==null?Double.NaN:selected.flightSec;
        double uncertainty=selected==null?Double.NaN:selected.flightUncertaintySec+selected.releaseUncertaintySec;
        Object[] values={revision,id,now,event,recipe==null?null:recipe.type,s.robot.getFieldX(),s.robot.getFieldY(),
                s.robot.getFieldVx(),s.robot.getFieldVy(),s.robot.getAngularVelocityRadPerSec(),ax,ay,alpha,
                s.target==null?null:s.target.targetId,s.target==null?Double.NaN:s.target.x,s.target==null?Double.NaN:s.target.y,
                s.target==null?Double.NaN:s.target.z,yawCommand,s.turretAngle,s.turretVelocity,c.pitchRad,
                f==null?Double.NaN:f.pitchRad,f==null?Double.NaN:f.pitchVelocity,c.gapMm,
                f==null?Double.NaN:f.gapMm,f==null?Double.NaN:f.gapVelocity,c.rpm,s.rpm,minimumRpm,c.feeder,s.staged,s.exit,
                release,flight,release+flight-uncertainty,release+flight+uncertainty,miss,c.reason,feederStarted,exitedAt,recoverySec,null,null,runId};
        add(row(values));
    }
    private static String row(Object[] values) {
        StringBuilder row=new StringBuilder();
        for(int i=0;i<values.length;i++) {if(i>0)row.append(',');row.append(cell(values[i]));}
        return row.toString();
    }
}
