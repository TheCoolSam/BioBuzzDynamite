package org.firstinspires.ftc.teamcode.turret;

/** Linear analog volts to one output-side turn. Health is applied before transfer. */
public final class AnalogAbsoluteConversion {
    private AnalogAbsoluteConversion() { }

    public static AbsoluteTurretSensor.Reading convert(double rawVolts, double adcMaxVolts,
            double zeroTurnVolts, double fullTurnVolts, AnalogVoltageHealth health,
            boolean wiringConfirmed) {
        double raw = rawVolts;
        String units = "V";
        if (health == null) {
            return new AbsoluteTurretSensor.Reading(raw, units, Double.NaN, "ANALOG HEALTH UNCONFIGURED");
        }
        String fault = health.check(rawVolts, adcMaxVolts);
        if (!fault.isEmpty()) {
            return new AbsoluteTurretSensor.Reading(raw, units, Double.NaN, fault);
        }
        if (!wiringConfirmed) {
            return new AbsoluteTurretSensor.Reading(raw, units, Double.NaN,
                    "ANALOG WIRING / TRANSFER NOT CONFIRMED");
        }
        if (!Double.isFinite(zeroTurnVolts) || !Double.isFinite(fullTurnVolts)
                || !Double.isFinite(adcMaxVolts) || zeroTurnVolts < 0 || fullTurnVolts <= zeroTurnVolts
                || fullTurnVolts > adcMaxVolts || rawVolts < zeroTurnVolts || rawVolts >= fullTurnVolts) {
            return new AbsoluteTurretSensor.Reading(raw, units, Double.NaN, "ANALOG RANGE / TRANSFER INVALID");
        }
        return new AbsoluteTurretSensor.Reading(raw, units,
                (rawVolts - zeroTurnVolts) / (fullTurnVolts - zeroTurnVolts), "");
    }
}
