package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.vision.pieces.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import java.util.*;

/** Coordinator owns the only read/write call site. Implementations enforce independent gates. */
public interface RobotIO {
    final class Snapshot {
        public final RobotState robot;
        public final double acquiredSec,turretAngle,turretVelocity,rpm;
        public final boolean healthy,entry,staged,exit;
        public final BallType entryType;
        public final TargetEstimate target;
        public final List<PieceObservation> pieces;
        public Snapshot(RobotState robot,double acquiredSec,double turretAngle,double turretVelocity,double rpm,
                boolean healthy,boolean entry,boolean staged,boolean exit,BallType entryType,
                TargetEstimate target,List<PieceObservation> pieces) {
            this.robot=robot;this.acquiredSec=acquiredSec;this.turretAngle=turretAngle;this.turretVelocity=turretVelocity;
            this.rpm=rpm;this.healthy=healthy;this.entry=entry;this.staged=staged;this.exit=exit;
            this.entryType=entryType;this.target=target;
            this.pieces=Collections.unmodifiableList(new ArrayList<PieceObservation>(pieces==null?Collections.<PieceObservation>emptyList():pieces));
        }
    }
    final class Command {
        public double fieldX,fieldY,turn,turretPower,rpm,intake,transfer,feeder;
        public double hood=Double.NaN,compression=Double.NaN;
        public boolean enabled;
        public String reason="DISABLED";
    }
    Snapshot read(double nowSec,double dtSec);
    void write(Command command);
    void stop();
}
