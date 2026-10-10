package org.firstinspires.ftc.teamcode.turret;

/** Acquisition seam: one output-side revolution, absolute at startup; never quadrature-only. */
public interface AbsoluteTurretSensor {
    Reading read();
    final class Reading {
        public final double raw;
        public final String units;
        public final double turns;
        public final String fault;
        public Reading(double raw, String units, double turns, String fault) {
            this.raw = raw; this.units = units; this.turns = turns; this.fault = fault;
        }
        public static Reading invalid(String fault) {
            return new Reading(Double.NaN, "unavailable", Double.NaN, fault);
        }
    }
}
