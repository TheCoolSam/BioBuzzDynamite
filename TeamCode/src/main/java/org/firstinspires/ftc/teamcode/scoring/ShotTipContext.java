package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallLoad;
import org.firstinspires.ftc.teamcode.vision.pieces.TargetEstimate;

/** Inputs for a measured tip estimate. A count-only estimate cannot certify a burst. */
public final class ShotTipContext {
    public final BallLoad carried;
    public final ShotSelector.Selection shot;
    public final TargetEstimate impact;
    public final HiveTracker.State hive;
    public final double cadenceSec,arrivalMinSec,arrivalMaxSec;
    public final List<BurstPolicy.Flight> pending;
    public ShotTipContext(BallLoad carried,ShotSelector.Selection shot,TargetEstimate impact,HiveTracker.State hive,
            double cadenceSec,double arrivalMinSec,double arrivalMaxSec,List<BurstPolicy.Flight> pending) {
        if(carried==null||shot==null||impact==null||hive==null||!Double.isFinite(cadenceSec)||cadenceSec<=0
                ||!Double.isFinite(arrivalMinSec)||!Double.isFinite(arrivalMaxSec)||arrivalMaxSec<arrivalMinSec||pending==null)
            throw new IllegalArgumentException("Shot, impact, hive and measured cadence required");
        this.carried=carried;this.shot=shot;this.impact=impact;this.hive=hive;this.cadenceSec=cadenceSec;
        this.arrivalMinSec=arrivalMinSec;this.arrivalMaxSec=arrivalMaxSec;
        this.pending=Collections.unmodifiableList(new ArrayList<BurstPolicy.Flight>(pending));
    }
}
