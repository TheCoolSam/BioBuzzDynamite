package org.firstinspires.ftc.teamcode.vision.pieces;

import java.util.*;

/** Owns camera I2C acquisition off the actuator thread. No actuator calls on this worker. */
public final class HuskyLensWorker implements AutoCloseable {
    public static final long INITIAL_BACKOFF_MS = 50;
    public static final long MAX_BACKOFF_MS = 2000;
    private final PieceObservationSource source;
    private final long epochNanos;
    private List<PieceObservation> observations = Collections.emptyList();
    private volatile boolean running;
    private Thread worker;
    private boolean healthy;
    private String lastError = "CAMERA WARMUP";
    private int restartCount;
    private long lastSuccessNanos;
    public HuskyLensWorker(PieceObservationSource source, long epochNanos) {
        if (source == null) throw new IllegalArgumentException("Camera source required");
        this.source = source;
        this.epochNanos = epochNanos;
    }
    public synchronized void start() {
        if (running) return;
        if (worker != null && worker.isAlive()) throw new IllegalStateException("Previous camera read still pending");
        running = true;
        healthy = false;
        lastError = "CAMERA WARMUP";
        worker = new Thread(new Runnable() {
            @Override public void run() {
                long backoffMs = INITIAL_BACKOFF_MS;
                while (running && !Thread.currentThread().isInterrupted()) {
                    try {
                        double requested = (System.nanoTime() - epochNanos) * 1e-9;
                        List<PieceObservation> read = source.read(requested);
                        synchronized (HuskyLensWorker.this) {
                            if (running) {
                                observations = Collections.unmodifiableList(new ArrayList<PieceObservation>(read));
                                healthy = true;
                                lastError = "";
                                lastSuccessNanos = System.nanoTime();
                            }
                        }
                        backoffMs = INITIAL_BACKOFF_MS;
                        Thread.sleep(INITIAL_BACKOFF_MS);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    } catch (RuntimeException e) {
                        synchronized (HuskyLensWorker.this) {
                            observations = Collections.emptyList();
                            healthy = false;
                            lastError = e.getClass().getSimpleName() + ": " + e.getMessage();
                            restartCount++;
                        }
                        try {
                            Thread.sleep(backoffMs);
                        } catch (InterruptedException interrupted) {
                            Thread.currentThread().interrupt();
                            break;
                        }
                        backoffMs = Math.min(MAX_BACKOFF_MS, backoffMs * 2);
                    }
                }
                synchronized (HuskyLensWorker.this) { running = false; }
            }
        }, "FloorVisionAcquisition");
        worker.setDaemon(true);
        worker.start();
    }
    /** Empty when the worker is closed or unhealthy. Consumers must treat that as no fresh data. */
    public synchronized List<PieceObservation> latest() {
        return healthy ? observations : Collections.<PieceObservation>emptyList();
    }
    public synchronized boolean healthy() { return healthy && running; }
    public synchronized String lastError() { return lastError; }
    public synchronized int restartCount() { return restartCount; }
    public synchronized double lastSuccessAgeSec() {
        return lastSuccessNanos == 0 ? Double.NaN : (System.nanoTime() - lastSuccessNanos) * 1e-9;
    }
    @Override public synchronized void close() {
        running = false;
        healthy = false;
        lastError = "CAMERA CLOSED";
        observations = Collections.emptyList();
        if (worker != null) worker.interrupt();
    }
}
