package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.scoring.ShooterCalibrationStorage;

/**
 * Dedicated Bench Tuner for BioBuzz Shooter & Hood assembly.
 * Configured specifically for:
 * - 1x Axon Servo for Hood Angle ("hood")
 * - 1x Axon Servo for Ball Compression ("compression")
 * - 1x Flywheel Motor ("flywheel")
 * - 1x Limelight 3A ("limelight")
 *
 * Keeps servos completely disabled (unpowered) during INIT to prevent jumping or twitching.
 * Sets calibrated Axon 500us - 2500us PWM range to eliminate jitter.
 */
@Config
@TeleOp(name = "Shooter & Hood Bench Tuner", group = "Bench")
public class TurretShooterBenchOpMode extends OpMode {

    // FTC Dashboard live-tunable variables
    public static double TARGET_RPM = 1400.0;
    public static double P = 15.0;
    public static double I = 0.0;
    public static double D = 0.5;
    public static double F = 12.5;

    public static double HOOD_POSITION = 0.35;
    public static double POLLEN_COMPRESSION = 0.30;
    public static double NECTAR_COMPRESSION = 0.50;
    public static String ACTIVE_BALL = "POLLEN"; // "POLLEN" or "NECTAR"
    public static double TICKS_PER_REV = 28.0;

    public static boolean SERVOS_ENABLED = true;
    public static boolean SAVE_CALIBRATION = false;
    public static int LIMELIGHT_PIPELINE = 0;

    private MultipleTelemetry multiTelemetry;
    private DcMotorEx flywheel;
    private Servo hood;
    private Servo compression;
    private Limelight3A limelight;

    private boolean flywheelEnabled = false;
    private boolean servosEngaged = false;
    private BallType activeBall = BallType.POLLEN;
    private String statusMessage = "Ready";

    private boolean previousCross = false;
    private boolean previousTriangle = false;
    private boolean previousSquare = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousLb = false;
    private boolean previousRb = false;
    private boolean previousOptions = false;

    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;
    private double lastF = Double.NaN;
    private double lastHood = Double.NaN;
    private double lastCompression = Double.NaN;

    // Axon official PWM frame range (500us - 2500us at 50Hz/20ms frame)
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500, 20000);

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Load saved calibration from disk
        ShooterCalibrationStorage.Data saved = ShooterCalibrationStorage.load();
        POLLEN_COMPRESSION = saved.pollenCompression;
        NECTAR_COMPRESSION = saved.nectarCompression;
        HOOD_POSITION = saved.hoodPosition;
        TARGET_RPM = saved.targetRpm;
        P = saved.p;
        I = saved.i;
        D = saved.d;
        F = saved.f;
        statusMessage = "Loaded calibration from storage";

        // Initialize Flywheel
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setPower(0);
            updatePidfCoefficients();
            multiTelemetry.addLine("Flywheel: Connected");
        } catch (Exception e) {
            multiTelemetry.addData("Flywheel Error", e.getMessage());
        }

        // Initialize Hood Axon Servo (Keep PWM DISABLED in init to prevent snapping/twitching)
        try {
            hood = hardwareMap.get(Servo.class, "hood");
            if (hood instanceof PwmControl) {
                ((PwmControl) hood).setPwmRange(AXON_PWM_RANGE);
                ((PwmControl) hood).setPwmDisable();
            }
            multiTelemetry.addLine("Hood Axon Servo: Connected (PWM Disabled until Start)");
        } catch (Exception e) {
            multiTelemetry.addData("Hood Servo Error", e.getMessage());
        }

        // Initialize Compression Axon Servo (Keep PWM DISABLED in init to prevent snapping/twitching)
        try {
            compression = hardwareMap.get(Servo.class, "compression");
            if (compression instanceof PwmControl) {
                ((PwmControl) compression).setPwmRange(AXON_PWM_RANGE);
                ((PwmControl) compression).setPwmDisable();
            }
            multiTelemetry.addLine("Compression Axon Servo: Connected (PWM Disabled until Start)");
        } catch (Exception e) {
            multiTelemetry.addData("Compression Servo Error", e.getMessage());
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
            multiTelemetry.addLine("Limelight 3A: Connected");
        } catch (Exception e) {
            multiTelemetry.addData("Limelight Error", e.getMessage());
        }

        multiTelemetry.addLine("--- READY ---");
        multiTelemetry.addLine("Servos are safely OFF in INIT. Press START to enable.");
        multiTelemetry.update();
    }

    @Override
    public void start() {
        // When START is pressed, engage servos cleanly
        if (SERVOS_ENABLED) {
            enableServos();
            servosEngaged = true;
        }
        statusMessage = "OpMode Running";
    }

    @Override
    public void loop() {
        // Emergency Stop: PS5 Circle or Gamepad B (instantly cuts power and relaxes servos)
        boolean btnStop = gamepad1.circle || gamepad1.b;
        if (btnStop) {
            flywheelEnabled = false;
            servosEngaged = false;
            disableServos();
            if (flywheel != null) flywheel.setPower(0);
            statusMessage = "EMERGENCY STOP (Servos & Motor Cut)";
            publishTelemetry(0.0, 0.0, 0.0, false);
            return;
        }

        // Check if Dashboard triggered a calibration save
        if (SAVE_CALIBRATION) {
            SAVE_CALIBRATION = false;
            saveCurrentCalibration();
        }

        // Save calibration: PS5 Options or Gamepad Start
        boolean btnSave = gamepad1.options || gamepad1.start;
        if (btnSave && !previousOptions) {
            saveCurrentCalibration();
        }
        previousOptions = btnSave;

        // Toggle Servos Power: PS5 Square or Gamepad X
        boolean btnSquare = gamepad1.square || gamepad1.x;
        if (btnSquare && !previousSquare) {
            servosEngaged = !servosEngaged;
            if (servosEngaged) {
                enableServos();
                statusMessage = "Servos Engaged";
            } else {
                disableServos();
                statusMessage = "Servos Relaxed (Unpowered)";
            }
        }
        previousSquare = btnSquare;

        // Toggle active ball size: PS5 Triangle or Gamepad Y
        boolean btnBallToggle = gamepad1.triangle || gamepad1.y;
        if (btnBallToggle && !previousTriangle) {
            if (activeBall == BallType.POLLEN) {
                activeBall = BallType.NECTAR;
                ACTIVE_BALL = "NECTAR";
            } else {
                activeBall = BallType.POLLEN;
                ACTIVE_BALL = "POLLEN";
            }
            statusMessage = "Active ball: " + activeBall.name();
        }
        previousTriangle = btnBallToggle;

        // Sync ACTIVE_BALL from Dashboard if changed
        if ("NECTAR".equalsIgnoreCase(ACTIVE_BALL) && activeBall != BallType.NECTAR) {
            activeBall = BallType.NECTAR;
        } else if ("POLLEN".equalsIgnoreCase(ACTIVE_BALL) && activeBall != BallType.POLLEN) {
            activeBall = BallType.POLLEN;
        }

        // Adjust Compression: D-Pad Left (-0.01) / Right (+0.01)
        if (gamepad1.dpad_left && !previousDpadLeft) {
            adjustActiveCompression(-0.01);
        }
        previousDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !previousDpadRight) {
            adjustActiveCompression(+0.01);
        }
        previousDpadRight = gamepad1.dpad_right;

        // Adjust Hood Position: L1 (-0.01) / R1 (+0.01)
        if (gamepad1.left_bumper && !previousLb) {
            HOOD_POSITION = Math.max(0.0, Math.min(1.0, HOOD_POSITION - 0.01));
        }
        previousLb = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !previousRb) {
            HOOD_POSITION = Math.max(0.0, Math.min(1.0, HOOD_POSITION + 0.01));
        }
        previousRb = gamepad1.right_bumper;

        // Adjust Target RPM: D-Pad Up (+50) / Down (-50)
        if (gamepad1.dpad_up && !previousDpadUp) {
            TARGET_RPM = Math.min(6000.0, TARGET_RPM + 50.0);
        }
        previousDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !previousDpadDown) {
            TARGET_RPM = Math.max(0.0, TARGET_RPM - 50.0);
        }
        previousDpadDown = gamepad1.dpad_down;

        // Toggle Flywheel: PS5 Cross or Gamepad A
        boolean btnCross = gamepad1.cross || gamepad1.a;
        if (btnCross && !previousCross) {
            flywheelEnabled = !flywheelEnabled;
            statusMessage = flywheelEnabled ? "Flywheel Spinning" : "Flywheel Stopped";
        }
        previousCross = btnCross;

        // Command Servos only when engaged
        if (servosEngaged) {
            if (hood != null && HOOD_POSITION != lastHood) {
                hood.setPosition(HOOD_POSITION);
                lastHood = HOOD_POSITION;
            }

            double currentComp = getActiveCompression();
            if (compression != null && currentComp != lastCompression) {
                compression.setPosition(currentComp);
                lastCompression = currentComp;
            }
        }

        // Apply PIDF if changed
        if (P != lastP || I != lastI || D != lastD || F != lastF) {
            updatePidfCoefficients();
        }

        // Flywheel Control
        double actualRpm = 0.0;
        if (flywheel != null) {
            if (flywheelEnabled) {
                double targetTicksPerSec = TARGET_RPM * (TICKS_PER_REV / 60.0);
                flywheel.setVelocity(targetTicksPerSec);
            } else {
                flywheel.setPower(0);
            }
            actualRpm = flywheel.getVelocity() * (60.0 / TICKS_PER_REV);
        }

        // Limelight Feedback
        double tx = 0.0, ty = 0.0;
        boolean hasTarget = false;
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                hasTarget = true;
                tx = result.getTx();
                ty = result.getTy();
            }
        }

        publishTelemetry(actualRpm, tx, ty, hasTarget);
    }

    private void enableServos() {
        if (hood instanceof PwmControl) {
            ((PwmControl) hood).setPwmEnable();
        }
        if (hood != null) {
            hood.setPosition(HOOD_POSITION);
            lastHood = HOOD_POSITION;
        }

        if (compression instanceof PwmControl) {
            ((PwmControl) compression).setPwmEnable();
        }
        if (compression != null) {
            double comp = getActiveCompression();
            compression.setPosition(comp);
            lastCompression = comp;
        }
    }

    private void disableServos() {
        if (hood instanceof PwmControl) {
            ((PwmControl) hood).setPwmDisable();
        }
        if (compression instanceof PwmControl) {
            ((PwmControl) compression).setPwmDisable();
        }
        lastHood = Double.NaN;
        lastCompression = Double.NaN;
    }

    private double getActiveCompression() {
        return activeBall == BallType.POLLEN ? POLLEN_COMPRESSION : NECTAR_COMPRESSION;
    }

    private void adjustActiveCompression(double delta) {
        if (activeBall == BallType.POLLEN) {
            POLLEN_COMPRESSION = Math.max(0.0, Math.min(1.0, POLLEN_COMPRESSION + delta));
        } else {
            NECTAR_COMPRESSION = Math.max(0.0, Math.min(1.0, NECTAR_COMPRESSION + delta));
        }
    }

    private void saveCurrentCalibration() {
        ShooterCalibrationStorage.Data data = new ShooterCalibrationStorage.Data();
        data.pollenCompression = POLLEN_COMPRESSION;
        data.nectarCompression = NECTAR_COMPRESSION;
        data.hoodPosition = HOOD_POSITION;
        data.targetRpm = TARGET_RPM;
        data.p = P;
        data.i = I;
        data.d = D;
        data.f = F;

        boolean success = ShooterCalibrationStorage.save(data);
        statusMessage = success ? "CALIBRATION SAVED TO DISK" : "SAVE FAILED (Check log)";
    }

    private void updatePidfCoefficients() {
        if (flywheel != null) {
            PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastP = P;
            lastI = I;
            lastD = D;
            lastF = F;
        }
    }

    private void publishTelemetry(double actualRpm, double tx, double ty, boolean hasTarget) {
        int rawTicks = 0;
        double rawVelocity = 0.0;
        double power = 0.0;
        if (flywheel != null) {
            rawTicks = flywheel.getCurrentPosition();
            rawVelocity = flywheel.getVelocity();
            power = flywheel.getPower();
        }

        // Graphing fields for FTC Dashboard
        multiTelemetry.addData("targetRpm", TARGET_RPM);
        multiTelemetry.addData("actualRpm", actualRpm);
        multiTelemetry.addData("rpmError", flywheelEnabled ? (TARGET_RPM - actualRpm) : 0.0);
        multiTelemetry.addData("Flywheel Enabled (press Cross/A)", flywheelEnabled);
        multiTelemetry.addData("Dashboard PIDF (P,I,D,F)", "%.2f, %.2f, %.2f, %.2f", P, I, D, F);
        if (flywheel != null) {
            PIDFCoefficients hub = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            multiTelemetry.addData("Hub PIDF readback", "%.2f, %.2f, %.2f, %.2f", hub.p, hub.i, hub.d, hub.f);
        }
        multiTelemetry.addData("flywheelPower", power);

        // Encoder Diagnostic Check
        multiTelemetry.addData("Encoder Raw Ticks", rawTicks);
        multiTelemetry.addData("Encoder Raw Vel (ticks/s)", "%.1f", rawVelocity);
        if (flywheelEnabled && power > 0.05 && rawVelocity == 0.0) {
            multiTelemetry.addLine(">> WARNING: Motor powered but 0 encoder pulses detected! Check 4-pin encoder cable! <<");
        }

        // Active Tuning State
        multiTelemetry.addData("STATUS", statusMessage);
        multiTelemetry.addData("Servos State", servosEngaged ? "ENGAGED" : "RELAXED (Unpowered)");
        multiTelemetry.addData("Active Ball", "%s (Compression: %.3f)", activeBall.name(), getActiveCompression());
        multiTelemetry.addData("Pollen Compression", "%.3f", POLLEN_COMPRESSION);
        multiTelemetry.addData("Nectar Compression", "%.3f", NECTAR_COMPRESSION);
        multiTelemetry.addData("Hood Position", "%.3f", HOOD_POSITION);

        // Limelight Alignment
        multiTelemetry.addData("Limelight Target Lock", hasTarget ? "LOCKED" : "SEARCHING");
        multiTelemetry.addData("Tag tx (Yaw deg)", "%.2f", tx);
        multiTelemetry.addData("Tag ty (Pitch deg)", "%.2f", ty);

        // Controls summary
        multiTelemetry.addLine("--- PS5 CONTROLS ---");
        multiTelemetry.addLine("[Square / \u25a1] Toggle Servos Power (Relax/Hold)");
        multiTelemetry.addLine("[Triangle / \u0394] Toggle Ball (POLLEN/NECTAR) | [D-Pad L/R] Compression +/-0.01");
        multiTelemetry.addLine("[L1 / R1] Hood Angle +/-0.01 | [D-Pad U/D] RPM +/-50 | [Cross / \u2715] Flywheel Toggle");
        multiTelemetry.addLine("[Options] Save Calibration to Disk | [Circle / \u25cb] Emergency STOP");

        multiTelemetry.update();
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        disableServos();
        if (limelight != null) {
            try { limelight.stop(); } catch (Exception ignored) {}
        }
    }
}
