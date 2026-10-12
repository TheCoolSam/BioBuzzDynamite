package org.firstinspires.ftc.teamcode.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * MANUAL bulk caching for every hub. Enable once, then {@link #beginLoop()}
 * exactly once per control cycle before any cached reads.
 */
public final class HubBulkCache {
    public interface Hub {
        void setManual();
        void clear();
    }

    private final List<Hub> hubs;
    private boolean enabled;
    private int clears;

    public HubBulkCache(List<Hub> hubs) {
        if(hubs==null)throw new IllegalArgumentException("Hub list required");
        this.hubs=Collections.unmodifiableList(new ArrayList<Hub>(hubs));
        for(Hub hub:this.hubs)if(hub==null)throw new IllegalArgumentException("Null hub");
    }

    public int hubCount() {return hubs.size();}
    public boolean enabled() {return enabled;}
    public int clears() {return clears;}

    public void enableManual() {
        for(int i=0;i<hubs.size();i++)hubs.get(i).setManual();
        enabled=true;
    }

    public void beginLoop() {
        if(!enabled)throw new IllegalStateException("MANUAL bulk caching must be enabled before the loop");
        for(int i=0;i<hubs.size();i++)hubs.get(i).clear();
        clears++;
    }
}
