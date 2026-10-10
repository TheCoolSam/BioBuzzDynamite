package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** REV motor translation, absolute acquisition, and final output interlock. No targeting logic. */
public final class RevTurretIO implements TurretIO {
    private final DcMotor motor;
    private final AbsoluteTurretSensor sensor;
    public final TurretEncoderSample sample = new TurretEncoderSample();
    public final TurretOutputSafety safety;
    private final boolean competition;
    public double appliedPower;
    public double acquisitionDtSec = Double.NaN;
    public String motorFault = "";
    private long lastReadNanos;
    private double lastDt = Double.NaN;

    public RevTurretIO(HardwareMap map) {
        this(map, TurretOutputSafety.benchDefaults(), false);
    }

    public RevTurretIO(HardwareMap map, TurretControlProfile profile) {
        this(map, competitionSafety(profile), true);
    }
    private static TurretOutputSafety competitionSafety(TurretControlProfile profile) {
        if (profile==null || !profile.measured || !TurretHardwareConfig.COMPETITION_TUNING_CONFIRMED
                || !TurretHardwareConfig.MOTOR_SIGN_VERIFIED) throw new IllegalStateException("Measured turret tune required");
        return new TurretOutputSafety(profile.PHYSICAL_MIN_RAD,profile.PHYSICAL_MAX_RAD,
                profile.OPERATING_MIN_RAD,profile.OPERATING_MAX_RAD,profile.OPERATING_MIN_RAD,
                profile.OPERATING_MAX_RAD,profile.MAX_MOTOR_POWER);
    }
    private RevTurretIO(HardwareMap map, TurretOutputSafety safety, boolean competition) {
        this.safety=safety;this.competition=competition;
        motor = map.get(DcMotor.class, TurretHardwareConfig.MOTOR_NAME);
        AbsoluteTurretSensor selected;
        try {
            motor.setPower(0);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            selected = selectSensor(map);
        } catch (RuntimeException failure) {
            motor.setPower(0);
            throw failure;
        }
        sensor = selected;
    }

    private static AbsoluteTurretSensor selectSensor(HardwareMap map) {
        if (TurretHardwareConfig.BACKEND == TurretHardwareConfig.Backend.UNSELECTED) {
            return () -> AbsoluteTurretSensor.Reading.invalid("ABSOLUTE ACQUISITION UNSELECTED");
        }
        if (TurretHardwareConfig.BACKEND == TurretHardwareConfig.Backend.PWM_ABSOLUTE) {
            // DigitalChannel.getState() is not a pulse-width capture API. No invented adapter.
            return () -> AbsoluteTurretSensor.Reading.invalid("PWM CAPTURE BACKEND NOT IMPLEMENTED");
        }
        if (!TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED) {
            return () -> AbsoluteTurretSensor.Reading.invalid("ANALOG WIRING / TRANSFER NOT CONFIRMED");
        }
        final AnalogInput input = map.get(AnalogInput.class, TurretHardwareConfig.ENCODER_NAME);
        return () -> {
            double raw = input.getVoltage();
            double low = TurretHardwareConfig.ANALOG_ZERO_TURN_VOLTS;
            double high = TurretHardwareConfig.ANALOG_FULL_TURN_VOLTS;
            double adcMax = input.getMaxVoltage();
            if (!Double.isFinite(low) || !Double.isFinite(high) || low < 0 || high <= low
                    || !Double.isFinite(adcMax) || high > adcMax
                    || !Double.isFinite(raw) || raw < low || raw >= high) {
                return new AbsoluteTurretSensor.Reading(raw, "V", Double.NaN,
                        "ANALOG RANGE / TRANSFER INVALID");
            }
            return new AbsoluteTurretSensor.Reading(raw, "V", (raw - low) / (high - low), "");
        };
    }

    public void read(double dt) {
        lastDt = dt;
        try {
            AbsoluteTurretSensor.Reading reading = sensor.read();
            long acquiredNanos = System.nanoTime();
            acquisitionDtSec = lastReadNanos == 0 ? dt : (acquiredNanos - lastReadNanos) * 1e-9;
            sample.update(reading, acquisitionDtSec, TurretHardwareConfig.ENCODER_SIGN,
                    TurretHardwareConfig.ENCODER_ZERO_OFFSET_RAD,
                    TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED,
                    TurretHardwareConfig.SENSOR_MAX_RATE_RAD_PER_SEC);
            if (!TurretEncoderSample.validDt(dt)) sample.fail("LOOP DT INVALID");
            if (sample.valid && (sample.angleRad < safety.physicalMin()
                    || sample.angleRad > safety.physicalMax())) {
                sample.fail("ANGLE OUTSIDE PHYSICAL RANGE");
            }
            lastReadNanos = acquiredNanos;
        } catch (RuntimeException failure) {
            sample.raw = sample.unoffsetRad = Double.NaN;
            acquisitionDtSec = Double.NaN;
            sample.fail("HARDWARE READ FAILURE: " + failure.getClass().getSimpleName());
            lastReadNanos = 0;
        }
        if (!sample.valid) stop();
    }

    /** No generic caller may bypass the continuous bench enable gates. */
    @Override public void setMotorPower(double demand) { command(demand, false, false, false); }

    public void command(double demand, boolean armed, boolean deadman, boolean emergencyStop) {
        double age = sampleAgeSec();
        boolean fresh = lastReadNanos != 0 && age >= 0 && age <= TurretHardwareConfig.MAX_DT_SEC;
        boolean configured = TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED
                && TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED
                && TurretHardwareConfig.ENCODER_SIGN_VERIFIED && TurretHardwareConfig.LIMITS_CONFIRMED
                && TurretHardwareConfig.BENCH_POWER_CONFIRMED
                && (!competition || (TurretHardwareConfig.COMPETITION_TUNING_CONFIRMED
                    && TurretHardwareConfig.MOTOR_SIGN_VERIFIED && TurretHardwareConfig.CLOSED_LOOP_ENABLED))
                && (TurretHardwareConfig.MOTOR_SIGN == 1 || TurretHardwareConfig.MOTOR_SIGN == -1);
        double power = safety.apply(demand, sample.angleRad, sample.valid && fresh,
                lastDt, armed && configured && motorFault.isEmpty(), deadman, emergencyStop);
        if (!configured && safety.reason.equals("NOT ARMED")) safety.reason = "HARDWARE CHECKLIST INCOMPLETE";
        write(power);
    }
    public double sampleAgeSec() {
        return lastReadNanos == 0 ? Double.NaN : (System.nanoTime() - lastReadNanos) * 1e-9;
    }
    private void write(double power) {
        try {
            motor.setPower(power * TurretHardwareConfig.MOTOR_SIGN);
            appliedPower = power;
        } catch (RuntimeException failure) {
            motorFault = "MOTOR WRITE FAILURE: " + failure.getClass().getSimpleName();
            appliedPower = Double.NaN; // Actual output cannot be confirmed if both writes fail.
            try { motor.setPower(0); appliedPower = 0; }
            catch (RuntimeException ignored) { motorFault += " / ZERO WRITE FAILED: USE PHYSICAL STOP"; }
        }
    }
    @Override public void stop() { write(0); }
    @Override public double getAngleRad() { return sample.angleRad; }
    @Override public double getVelocityRadPerSec() { return sample.velocityRadPerSec; }
}
