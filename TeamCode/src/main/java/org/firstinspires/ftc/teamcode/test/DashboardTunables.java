package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * FTC Dashboard live-tuning that can prove it is working.
 *
 * <p>Each tunable is registered through an explicit {@link ValueProvider}, so the robot sees every
 * Dashboard edit arrive (count, name, value, age) and can show that on the Driver Station. Changes
 * made on the gamepad or in code are pushed back so the Dashboard panel never shows stale numbers.
 *
 * <p>Usage: create tunables, call {@link #register()} in init(), call {@link #pushIfDirty()} every
 * loop, and call {@link #unregister()} in stop().
 *
 * <p>In the Dashboard config panel: type the new value and press ENTER (or click the check mark).
 * Until then the edit has not been sent to the robot.
 */
public final class DashboardTunables {
    private final String category;
    private final List<Entry> entries = new ArrayList<Entry>();
    private final AtomicInteger edits = new AtomicInteger();
    private volatile String lastEdit = "none yet";
    private volatile long lastEditNanos = 0;
    private volatile boolean dirty = false;
    private long lastPushNanos = 0;
    private boolean registered = false;
    private String status = "NOT REGISTERED";

    private static final class Entry {
        final String name;
        final ValueProvider<?> provider;
        Entry(String name, ValueProvider<?> provider) { this.name = name; this.provider = provider; }
    }

    public DashboardTunables(String category) { this.category = category; }

    public final class Num implements ValueProvider<Double> {
        public final String name;
        private volatile double value;

        private Num(String name, double initial) { this.name = name; this.value = initial; }

        public double v() { return value; }

        /** Set from the gamepad or code. Only marks the Dashboard stale when the value really changed. */
        public void local(double x) {
            if (x != value) {
                value = x;
                dirty = true;
            }
        }

        @Override public Double get() { return value; }

        @Override public void set(Double x) {
            if (x == null || Double.isNaN(x) || Double.isInfinite(x)) return;
            value = x;
            noteEdit(name + " = " + x);
        }
    }

    public final class Flag implements ValueProvider<Boolean> {
        public final String name;
        private volatile boolean value;

        private Flag(String name, boolean initial) { this.name = name; this.value = initial; }

        public boolean v() { return value; }

        public void local(boolean x) {
            if (x != value) {
                value = x;
                dirty = true;
            }
        }

        @Override public Boolean get() { return value; }

        @Override public void set(Boolean x) {
            if (x == null) return;
            value = x;
            noteEdit(name + " = " + x);
        }
    }

    private void noteEdit(String text) {
        edits.incrementAndGet();
        lastEdit = text;
        lastEditNanos = System.nanoTime();
    }

    public Num num(String name, double initial) {
        Num n = new Num(name, initial);
        entries.add(new Entry(name, n));
        return n;
    }

    public Flag flag(String name, boolean initial) {
        Flag f = new Flag(name, initial);
        entries.add(new Entry(name, f));
        return f;
    }

    /** Registers every tunable. Returns a one-line status for telemetry. Never throws. */
    @SuppressWarnings({"unchecked", "rawtypes"})
    public String register() {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) {
            registered = false;
            status = "NOT RUNNING (FtcDashboard.getInstance() == null). Edits cannot arrive.";
            return status;
        }
        try {
            unregister();
            for (Entry e : entries) dash.addConfigVariable(category, e.name, (ValueProvider) e.provider);
            registered = true;
            dash.updateConfig();
            status = dash.isEnabled()
                    ? "ENABLED, panel \"" + category + "\""
                    : "REGISTERED but dash.isEnabled() == false";
        } catch (RuntimeException failure) {
            registered = false;
            status = "REGISTRATION FAILED: " + failure;
        }
        return status;
    }

    public void unregister() {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) return;
        for (Entry e : entries) {
            try { dash.removeConfigVariable(category, e.name); } catch (RuntimeException ignored) { }
        }
        registered = false;
    }

    /** Push gamepad/code changes to the Dashboard panel. Rate limited; cheap when nothing changed. */
    public void pushIfDirty() {
        if (!registered || !dirty) return;
        long now = System.nanoTime();
        if (now - lastPushNanos < 150_000_000L) return;
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) return;
        dirty = false;
        lastPushNanos = now;
        try { dash.updateConfig(); } catch (RuntimeException ignored) { }
    }

    public String status() { return status; }

    public boolean isRegistered() { return registered; }

    public int editCount() { return edits.get(); }

    /** "3 edits (last: P = 2.0, 4.1 s ago)" for the Driver Station. */
    public String editSummary() {
        if (lastEditNanos == 0) return edits.get() + " edits (none yet)";
        double ageSec = (System.nanoTime() - lastEditNanos) / 1e9;
        return String.format("%d edits (last: %s, %.1f s ago)", edits.get(), lastEdit, ageSec);
    }
}
