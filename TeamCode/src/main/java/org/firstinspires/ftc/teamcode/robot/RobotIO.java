package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import java.util.*;
import org.firstinspires.ftc.teamcode.scoring.LauncherFeedback;

/** Coordinator owns the only read/write call site. Implementations enforce independent gates. */
public interface RobotIO {
    final class Health {
        public final boolean drive,localization,yaw,launcher,inventory;
        public Health(boolean drive,boolean localization,boolean yaw,boolean launcher,boolean inventory) {
            this.drive=drive;this.localization=localization;this.yaw=yaw;this.launcher=launcher;this.inventory=inventory;
        }
        public boolean allHealthy() {return drive&&localization&&yaw&&launcher&&inventory;}
        @Override public String toString() {return "drive="+drive+" localization="+localization+" yaw="+yaw+" launcher="+launcher+" inventory="+inventory;}
    }
    final class Snapshot {
        public final Health health;
        public final RobotState robot;
        public final LauncherFeedback launcher;
        public final double acquiredSec,turretAngle,turretVelocity,rpm;
        public final boolean healthy,entry,staged,exit;
        public final BallType entryType;
        public final TargetEstimate target;
        public final List<PieceObservation> pieces;
        public Snapshot(RobotState robot,double acquiredSec,double turretAngle,double turretVelocity,double rpm,
                boolean healthy,boolean entry,boolean staged,boolean exit,BallType entryType,
                TargetEstimate target,List<PieceObservation> pieces) {
            this(robot,acquiredSec,turretAngle,turretVelocity,rpm,healthy,entry,staged,exit,entryType,target,pieces,null);
        }
        public Snapshot(RobotState robot,double acquiredSec,double turretAngle,double turretVelocity,double rpm,
                boolean healthy,boolean entry,boolean staged,boolean exit,BallType entryType,
                TargetEstimate target,List<PieceObservation> pieces,LauncherFeedback launcher) {
            this(robot,acquiredSec,turretAngle,turretVelocity,rpm,new Health(healthy,healthy,healthy,healthy,healthy),entry,staged,exit,entryType,target,pieces,launcher);
        }
        public Snapshot(RobotState robot,double acquiredSec,double turretAngle,double turretVelocity,double rpm,
                Health health,boolean entry,boolean staged,boolean exit,BallType entryType,
                TargetEstimate target,List<PieceObservation> pieces,LauncherFeedback launcher) {
            if(health==null)throw new IllegalArgumentException("Subsystem health required");
            this.health=health;
            this.launcher=launcher;
            this.robot=robot;this.acquiredSec=acquiredSec;this.turretAngle=turretAngle;this.turretVelocity=turretVelocity;
            this.rpm=rpm;this.healthy=health.allHealthy();this.entry=entry;this.staged=staged;this.exit=exit;
            this.entryType=entryType;this.target=target;
            this.pieces=Collections.unmodifiableList(new ArrayList<PieceObservation>(pieces==null?Collections.<PieceObservation>emptyList():pieces));
        }
    }
    final class Command {
        public double fieldX,fieldY,turn,turretPower,rpm,intake,transfer,feeder;
        public double hood=Double.NaN,compression=Double.NaN,pitchRad=Double.NaN,gapMm=Double.NaN;
        public double acquiredSec;
        public boolean enabled;
        public String reason="DISABLED";
    }
    Snapshot read(double nowSec,double dtSec);
    void write(Command command);
    void stop();
    /** Recovery is opt-in and must use independent, healthy drive outputs. */
    default boolean enterDriveRecovery() {stop();return false;}
    default Snapshot readDriveRecovery(double now,double dt) {return null;}
    default void writeDriveRecovery(Command command,double dt) {stop();}
}
