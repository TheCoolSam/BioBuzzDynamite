package org.firstinspires.ftc.teamcode.robot;

/**
 * Consecutive-sample debounce for split subsystem health. One stale sample or one
 * long loop degrades only the affected outputs; a second consecutive failure latches
 * that subsystem. Drive remains commandable while non-drive subsystems are degraded
 * or latched. Drive and timing latches stay sticky (fail-closed) until a deliberate
 * reset; non-drive latches clear after consecutive healthy samples.
 */
public final class HealthWatch {
    public static final int LATCH_AFTER=2;
    public static final int CLEAR_AFTER=2;

    public static final class View {
        public boolean driveCommandable=true,localizationTrusted=true,yawCommandable=true,
                launcherCommandable=true,inventoryTrusted=true,timingOk=true;
        public boolean driveLatched,timingLatched,localizationLatched,yawLatched,launcherLatched,inventoryLatched;
        public String reason="HEALTHY";
        public boolean anyLatched() {
            return driveLatched||timingLatched||localizationLatched||yawLatched||launcherLatched||inventoryLatched;
        }
    }

    private int driveBad,timingBad,locBad,yawBad,launchBad,invBad;
    private int locGood,yawGood,launchGood,invGood;
    private boolean driveLatched,timingLatched,locLatched,yawLatched,launchLatched,invLatched;

    public void reset() {
        driveBad=timingBad=locBad=yawBad=launchBad=invBad=0;
        locGood=yawGood=launchGood=invGood=0;
        driveLatched=timingLatched=locLatched=yawLatched=launchLatched=invLatched=false;
    }

    public View update(RobotIO.Health health,boolean timingOk) {
        View view=new View();
        boolean drive=health!=null&&health.drive;
        boolean loc=health!=null&&health.localization;
        boolean yaw=health!=null&&health.yaw;
        boolean launcher=health!=null&&health.launcher;
        boolean inventory=health!=null&&health.inventory;
        driveLatched=sticky(drive,driveBad,driveLatched);
        driveBad=drive?0:driveBad+1;
        timingLatched=sticky(timingOk,timingBad,timingLatched);
        timingBad=timingOk?0:timingBad+1;
        locLatched=recoverable(loc,locBad,locGood,locLatched);
        locBad=loc?0:locBad+1;locGood=loc?locGood+1:0;
        yawLatched=recoverable(yaw,yawBad,yawGood,yawLatched);
        yawBad=yaw?0:yawBad+1;yawGood=yaw?yawGood+1:0;
        launchLatched=recoverable(launcher,launchBad,launchGood,launchLatched);
        launchBad=launcher?0:launchBad+1;launchGood=launcher?launchGood+1:0;
        invLatched=recoverable(inventory,invBad,invGood,invLatched);
        invBad=inventory?0:invBad+1;invGood=inventory?invGood+1:0;
        view.driveLatched=driveLatched;
        view.timingLatched=timingLatched;
        view.localizationLatched=locLatched;
        view.yawLatched=yawLatched;
        view.launcherLatched=launchLatched;
        view.inventoryLatched=invLatched;
        view.timingOk=timingOk&&!timingLatched;
        view.driveCommandable=drive&&timingOk&&!driveLatched&&!timingLatched;
        view.localizationTrusted=loc&&timingOk&&!locLatched;
        view.yawCommandable=yaw&&timingOk&&!yawLatched;
        view.launcherCommandable=launcher&&timingOk&&!launchLatched;
        view.inventoryTrusted=inventory&&!invLatched;
        if(!drive)view.reason=driveLatched?"DRIVE FAULT LATCHED":"DRIVE SAMPLE DROPPED";
        else if(!timingOk)view.reason=timingLatched?"LOOP TIMING FAULT LATCHED":"LOOP TIMING SAMPLE DROPPED";
        else if(!loc)view.reason=locLatched?"LOCALIZATION FAULT LATCHED":"LOCALIZATION SAMPLE DROPPED";
        else if(!yaw)view.reason=yawLatched?"YAW FAULT LATCHED":"YAW SAMPLE DROPPED";
        else if(!launcher)view.reason=launchLatched?"LAUNCHER FAULT LATCHED":"LAUNCHER SAMPLE DROPPED";
        else if(!inventory)view.reason=invLatched?"INVENTORY FAULT LATCHED":"INVENTORY SAMPLE DROPPED";
        else view.reason="HEALTHY";
        return view;
    }

    private static boolean sticky(boolean healthy,int bad,boolean latched) {
        return latched||(!healthy&&bad+1>=LATCH_AFTER);
    }

    private static boolean recoverable(boolean healthy,int bad,int good,boolean latched) {
        if(!healthy&&bad+1>=LATCH_AFTER)return true;
        if(latched&&healthy&&good+1>=CLEAR_AFTER)return false;
        return latched;
    }
}
