package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.*;

/** Ordered confirmed inventory plus capacity reservations. All mutation is event driven. */
public final class PieceInventory {
    private final int capacity;
    private final List<BallType> queue = new ArrayList<BallType>();
    private final Map<Long,BallType> pending = new LinkedHashMap<Long,BallType>();
    private long nextId=1, revision;
    private boolean confirmed=true;
    public PieceInventory(int capacity) {
        if (capacity<=0 || capacity>4) throw new IllegalArgumentException("Capacity must be 1..4");
        this.capacity=capacity;
    }
    public long reserve(BallType type) {
        if (!confirmed||type==null || controlledCount()>=capacity) return -1;
        long id=nextId++; pending.put(id,type); revision++; return id;
    }
    public boolean confirmEntry(long reservation) {
        BallType type=pending.remove(reservation);
        if (type==null) return false;
        queue.add(type); revision++; return true;
    }
    public void cancelReservation(long reservation) { if (pending.remove(reservation)!=null) revision++; }
    public boolean confirmExit(BallType expected) {
        if (queue.isEmpty() || queue.get(0)!=expected) return false;
        queue.remove(0); revision++; return true;
    }
    public BallType first() { return !confirmed||queue.isEmpty()?null:queue.get(0); }
    public int controlledCount() { return queue.size()+pending.size(); }
    public boolean canIntake() { return confirmed&&controlledCount()<capacity; }
    public boolean isConfirmed() {return confirmed;}
    public void markUncertain() {if(confirmed){confirmed=false;revision++;}}
    public long revision() { return revision; }
    public List<BallType> pieces() { return Collections.unmodifiableList(new ArrayList<BallType>(queue)); }
    public BallLoad load() {
        int pollen=0,nectar=0;
        for (BallType type:queue) { if (type==BallType.POLLEN) pollen++; else nectar++; }
        return new BallLoad(pollen,nectar);
    }
    /** Explicit operator/sensor reconciliation only; clearing counts is not jam recovery. */
    public void reconcile(List<BallType> measured) {
        if (measured==null || measured.size()>capacity || measured.contains(null)) throw new IllegalArgumentException("Confirmed queue required");
        queue.clear();queue.addAll(measured);pending.clear();confirmed=true;revision++;
    }
}
