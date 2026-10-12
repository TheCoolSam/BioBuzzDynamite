package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

/** Axon CR-servo translation, absolute acquisition, and final output interlock. No targeting logic. */
public final class RevTurretIO implements TurretIO {
    private final CRServo servo;
    private final PwmControl pwm;
    private final AxonCrCalibration calibration;
    private final AbsoluteTurretSensor sensor;
    private final AnalogVoltageHealth analogHealth;
    public final TurretEncoderSample sample = new TurretEncoderSample();
    public final TurretOutputSafety safety;
    private final boolean competition;
    public double appliedPower;
    public double appliedPulseUs = Double.NaN;
    public boolean pwmEnabled = true;
    public double acquisitionDtSec = Double.NaN;
    public String motorFault = "";
    public TurretCrCommand lastCommand = TurretCrCommand.stop(null);
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
        calibration = TurretHardwareConfig.CR_CALIBRATION;
        analogHealth = AnalogVoltageHealth.fromTurretConfig();
        boolean crReady = calibration != null && TurretHardwareConfig.CR_MODE_AND_NEUTRAL_VERIFIED;
        if (competition && !crReady) {
            throw new IllegalStateException("Verify Axon CR firmware, pulse calibration and neutral before competition");
        }
        CRServo mapped = null;
        PwmControl mappedPwm = null;
        try {
            mapped = map.get(CRServo.class, TurretHardwareConfig.SERVO_NAME);
            if (mapped instanceof PwmControl) mappedPwm = (PwmControl) mapped;
        } catch (RuntimeException missing) {
            if (crReady) throw missing;
        }
        if (crReady) {
            if (mapped == null || mappedPwm == null) {
                throw new IllegalStateException("Calibrated CR PWM control required");
            }
        }
        servo = mapped;
        pwm = mappedPwm;
        AbsoluteTurretSensor selected;
        try {
            if (servo != null) servo.setDirection(DcMotorSimple.Direction.FORWARD);
            if (crReady) {
                pwm.setPwmRange(new PwmControl.PwmRange(calibration.lowUs, calibration.highUs, calibration.frameUs));
            }
            apply(TurretCrCommand.stop(calibration));
            selected = selectSensor(map);
        } catch (RuntimeException failure) {
            apply(TurretCrCommand.stop(calibration));
            throw failure;
        }
        sensor = selected;
    }

    private AbsoluteTurretSensor selectSensor(HardwareMap map) {
        if (TurretHardwareConfig.BACKEND == TurretHardwareConfig.Backend.UNSELECTED) {
            return new AbsoluteTurretSensor() {
                @Override public AbsoluteTurretSensor.Reading read() {
                    return AbsoluteTurretSensor.Reading.invalid("ABSOLUTE ACQUISITION UNSELECTED");
                }
            };
        }
        if (TurretHardwareConfig.BACKEND == TurretHardwareConfig.Backend.PWM_ABSOLUTE) {
            // DigitalChannel.getState() is not a pulse-width capture API. No invented adapter.
            return new AbsoluteTurretSensor() {
                @Override public AbsoluteTurretSensor.Reading read() {
                    return AbsoluteTurretSensor.Reading.invalid("PWM CAPTURE BACKEND NOT IMPLEMENTED");
                }
            };
        }
        final AnalogInput input = map.get(AnalogInput.class, TurretHardwareConfig.ENCODER_NAME);
        analogHealth.reset();
        return new AbsoluteTurretSensor() {
            @Override public AbsoluteTurretSensor.Reading read() {
                return AnalogAbsoluteConversion.convert(input.getVoltage(), input.getMaxVoltage(),
                        TurretHardwareConfig.ANALOG_ZERO_TURN_VOLTS,
                        TurretHardwareConfig.ANALOG_FULL_TURN_VOLTS,
                        analogHealth,
                        TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED);
            }
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
        boolean configured = crDriveConfigured();
        double power = safety.apply(demand, sample.angleRad, sample.velocityRadPerSec, sample.valid && fresh,
                lastDt, armed && configured && motorFault.isEmpty(), deadman, emergencyStop);
        if (!configured && safety.reason.equals("NOT ARMED")) safety.reason = "HARDWARE CHECKLIST INCOMPLETE";
        if (safety.directionFault()) motorFault = "WRONG DIRECTION / STALL TRIP";
        if (power == 0) apply(TurretCrCommand.stop(calibration));
        else apply(TurretCrCommand.drive(calibration, power));
        appliedPower = power;
    }

    public void commandMeasurementPulse(double pulseUs, boolean armed, boolean deadman, boolean emergencyStop) {
        double age = sampleAgeSec();
        boolean fresh = lastReadNanos != 0 && age >= 0 && age <= TurretHardwareConfig.MAX_DT_SEC;
        double synthetic = 0;
        if (Double.isFinite(pulseUs) && Double.isFinite(TurretHardwareConfig.CR_MEASURE_START_US)
                && pulseUs != TurretHardwareConfig.CR_MEASURE_START_US) {
            synthetic = Math.signum(pulseUs - TurretHardwareConfig.CR_MEASURE_START_US)
                    * TurretHardwareConfig.DIRECTION_TRIP_MIN_POWER;
        }
        double allowed = safety.apply(synthetic, sample.angleRad, sample.velocityRadPerSec, sample.valid && fresh,
                lastDt, armed && motorFault.isEmpty() && TurretHardwareConfig.crMeasureWindowConfigured(),
                deadman, emergencyStop);
        if (safety.directionFault()) motorFault = "WRONG DIRECTION / STALL TRIP";
        if (allowed == 0 || !armed || !deadman || emergencyStop) {
            apply(TurretCrCommand.stop(calibration));
            appliedPower = 0;
            return;
        }
        TurretCrCommand command = TurretCrCommand.measure(pulseUs,
                TurretHardwareConfig.CR_MEASURE_MIN_US, TurretHardwareConfig.CR_MEASURE_MAX_US,
                TurretHardwareConfig.CR_MEASURE_FRAME_US);
        apply(command);
        appliedPower = command.writesOutput() ? synthetic : 0;
    }

    public boolean crDriveConfigured() {
        return TurretHardwareConfig.CR_MODE_AND_NEUTRAL_VERIFIED
                && calibration != null
                && calibration.sign == TurretHardwareConfig.MOTOR_SIGN
                && TurretHardwareConfig.WIRING_AND_TRANSFER_CONFIRMED
                && TurretHardwareConfig.OUTPUT_SIDE_ONE_TO_ONE_CONFIRMED
                && TurretHardwareConfig.ENCODER_SIGN_VERIFIED && TurretHardwareConfig.LIMITS_CONFIRMED
                && TurretHardwareConfig.BENCH_POWER_CONFIRMED
                && Double.isFinite(TurretHardwareConfig.CR_CALIBRATION.validatedSignTestPower(
                        TurretHardwareConfig.SIGN_TEST_POWER, TurretHardwareConfig.BENCH_MAX_POWER))
                && (!competition || (TurretHardwareConfig.COMPETITION_TUNING_CONFIRMED
                    && TurretHardwareConfig.MOTOR_SIGN_VERIFIED && TurretHardwareConfig.CLOSED_LOOP_ENABLED))
                && (TurretHardwareConfig.MOTOR_SIGN == 1 || TurretHardwareConfig.MOTOR_SIGN == -1);
    }

    public double sampleAgeSec() {
        return lastReadNanos == 0 ? Double.NaN : (System.nanoTime() - lastReadNanos) * 1e-9;
    }

    private void apply(TurretCrCommand command) {
        lastCommand = command;
        if (servo == null) {
            appliedPulseUs = Double.NaN;
            pwmEnabled = false;
            appliedPower = 0;
            return;
        }
        try {
            if (pwm != null) {
                if (command.pwmEnabled) pwm.setPwmEnable();
                if (Double.isFinite(command.sdkPower)) servo.setPower(command.sdkPower);
                else if (Double.isFinite(command.pulseUs)) pwm.setPulseWidth(command.pulseUs);
                if (!command.pwmEnabled) {
                    disablePwm();
                }
                pwmEnabled = command.pwmEnabled;
            } else {
                if (Double.isFinite(command.sdkPower)) servo.setPower(command.sdkPower);
                else servo.setPower(0);
                pwmEnabled = true;
            }
            appliedPulseUs = command.pulseUs;
        } catch (RuntimeException failure) {
            motorFault = "CR SERVO WRITE FAILURE: " + failure.getClass().getSimpleName();
            appliedPower = Double.NaN;
            appliedPulseUs = Double.NaN;
            try {
                if (calibration != null) servo.setPower(calibration.sdkPower(0));
                disablePwm();
                appliedPower = 0;
                pwmEnabled = false;
            } catch (RuntimeException ignored) {
                motorFault += " / ZERO WRITE FAILED: USE PHYSICAL STOP";
            }
        }
    }

    private void disablePwm() {
        if (pwm != null) {
            pwm.setPwmDisable();
            return;
        }
        if (servo != null && servo.getController() instanceof ServoControllerEx) {
            ((ServoControllerEx) servo.getController()).setServoPwmDisable(servo.getPortNumber());
        }
    }

    @Override public void stop() {
        apply(TurretCrCommand.stop(calibration));
        appliedPower = 0;
    }
    @Override public double getAngleRad() { return sample.angleRad; }
    @Override public double getVelocityRadPerSec() { return sample.velocityRadPerSec; }
}
