package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.*;

/** Owns camera I2C acquisition off the actuator thread. No actuator calls on this worker. */
public final class HuskyLensWorker implements AutoCloseable {
    private final PieceObservationSource source;
    private final long epochNanos;
    private volatile List<PieceObservation> observations=Collections.emptyList();
    private volatile boolean running;
    private Thread worker;
    public HuskyLensWorker(PieceObservationSource source,long epochNanos) {
        if(source==null)throw new IllegalArgumentException("Camera source required");this.source=source;this.epochNanos=epochNanos;
    }
    public synchronized void start() {
        if(running)return;
        if(worker!=null&&worker.isAlive())throw new IllegalStateException("Previous camera read still pending");
        running=true;
        worker=new Thread(()->{
            while(running&&!Thread.currentThread().isInterrupted()) {
                try {
                    double requested=(System.nanoTime()-epochNanos)*1e-9;
                    List<PieceObservation> read=source.read(requested);
                    synchronized(HuskyLensWorker.this) {
                        if(running)observations=Collections.unmodifiableList(new ArrayList<PieceObservation>(read));
                    }
                    Thread.sleep(50);
                } catch(InterruptedException e) {Thread.currentThread().interrupt();break;}
                catch(RuntimeException e) {observations=Collections.emptyList();break;}
            }
            running=false;
        },"FloorVisionAcquisition");
        worker.setDaemon(true);worker.start();
    }
    public List<PieceObservation> latest() {return observations;}
    @Override public synchronized void close() {
        running=false;observations=Collections.emptyList();if(worker!=null)worker.interrupt();
    }
}
