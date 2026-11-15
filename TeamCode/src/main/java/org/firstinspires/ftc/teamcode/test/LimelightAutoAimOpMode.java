package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.turret.AimPoint;
import org.firstinspires.ftc.teamcode.turret.RevTurretIO;
import org.firstinspires.ftc.teamcode.turret.TurretBenchSession;
import org.firstinspires.ftc.teamcode.turret.VisionAimController;

import java.util.List;

/**
 * Limelight auto-aim bench. Works today without the turret attached.
 *
 * <p><b>What it proves with no turret:</b> the Limelight is connected, the AprilTag pipeline sees the
 * tag you pick, tx has the sign you expect, the controller output is sensible, and Dashboard edits
 * arrive. Hold a tag in front of the camera and move it left and right: tx and the commanded demand
 * must follow.
 *
 * <p><b>What it does with the turret:</b> the demand goes through {@link RevTurretIO#command}, the same
 * output safety gate as the Turret Hardware Bench (arming, deadman, limits, power cap, direction trip,
 * e-stop). That gate refuses all output until {@code TurretHardwareConfig} is filled in from the
 * Turret Hardware Bench measurements. This OpMode does not and cannot bypass it. Until then the
 * telemetry shows exactly what is blocking output.
 *
 * <p>Assumes the Limelight is mounted on the turret so that tx == 0 means the shooter is aimed.
 *
 * <p>Controls: A arm (toggle) | hold RIGHT BUMPER = deadman | B = e-stop (latched) |
 * BACK+START clears e-stop | LB flips AIM_SIGN | D-pad U/D = kP, L/R = kD.
 */
@TeleOp(name = "Limelight Auto Aim", group = "Bench")
public class LimelightAutoAimOpMode extends OpMode {
    private static final String LIMELIGHT_NAME = "limelight";

    private final DashboardTunables dash = new DashboardTunables("Auto Aim");
    private final DashboardTunables.Num kP = dash.num("AIM_KP", 0.012);
    private final DashboardTunables.Num kD = dash.num("AIM_KD", 0.0004);
    private final DashboardTunables.Num maxPower = dash.num("AIM_MAX_POWER", 0.15);
    private final DashboardTunables.Num deadband = dash.num("AIM_DEADBAND_DEG", 0.5);
    private final DashboardTunables.Num lockDeg = dash.num("AIM_LOCK_DEG", 1.0);
    private final DashboardTunables.Num lockFrames = dash.num("AIM_LOCK_FRAMES", 5);
    private final DashboardTunables.Num filterAlpha = dash.num("AIM_FILTER_ALPHA", 0.5);
    private final DashboardTunables.Num aimSign = dash.num("AIM_SIGN", 1);
    private final DashboardTunables.Num aimOffset = dash.num("AIM_OFFSET_DEG", 0.0);
    private final DashboardTunables.Num tagId = dash.num("TAG_ID_(-1_any)", -1);
    private final DashboardTunables.Num pipeline = dash.num("PIPELINE", 0);
    private final DashboardTunables.Num maxStaleMs = dash.num("MAX_STALE_MS", 100);
    private final DashboardTunables.Flag useResultTx = dash.flag("USE_NON_TAG_TX", false);
    // Aim at a point defined in the TAG's frame (inches), e.g. the top of the hive opening, instead of the tag centre.
    // Signs of the tag axes are verified on the robot: change one number and watch "Aim point elevation".
    private final DashboardTunables.Flag aimAtPoint = dash.flag("AIM_AT_POINT", false);
    private final DashboardTunables.Num pointX = dash.num("POINT_TAG_X_IN", 0.0);
    private final DashboardTunables.Num pointY = dash.num("POINT_TAG_Y_IN", 0.0);
    private final DashboardTunables.Num pointZ = dash.num("POINT_TAG_Z_IN", 0.0);

    private final VisionAimController aim = new VisionAimController();
    private final TurretBenchSession.Readiness readiness = TurretBenchSession.Readiness.configured();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();

    private MultipleTelemetry multi;
    private Limelight3A limelight;
    private RevTurretIO io;
    private String limelightFault = "";
    private String turretFault = "";
    private String dashStatus = "NOT CHECKED";

    private boolean armed = false;
    private boolean emergencyStop = false;
    private boolean pA, pLb, pUp, pDown, pLeft, pRight;
    private int appliedPipeline = Integer.MIN_VALUE;

    // vision bookkeeping
    private double lastResultTimestamp = Double.NaN;
    private double lastNewFrameSec = Double.NaN;
    private double fps = 0;
    private int framesSeen = 0;

    @Override
    public void init() {
        FtcDashboard ftcDash = FtcDashboard.getInstance();
        multi = new MultipleTelemetry(telemetry, ftcDash != null ? ftcDash.getTelemetry() : telemetry);
        telemetry.setMsTransmissionInterval(50);
        dashStatus = dash.register();

        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.setPollRateHz(100);
            applyPipeline();
            limelight.start();
        } catch (RuntimeException failure) {
            limelight = null;
            limelightFault = "LIMELIGHT \"" + LIMELIGHT_NAME + "\" NOT AVAILABLE: " + failure.getMessage();
        }
        try {
            io = new RevTurretIO(hardwareMap);
        } catch (RuntimeException failure) {
            io = null;
            turretFault = "TURRET IO UNAVAILABLE: " + failure.getMessage();
        }
        clock.reset();
        loopTimer.reset();
    }

    @Override public void init_loop() { cycle(false); }

    @Override
    public void start() {
        armed = false;
        emergencyStop = false;
        if (io != null) io.stop();
        loopTimer.reset();
    }

    @Override public void loop() { cycle(true); }

    @Override
    public void stop() {
        if (io != null) io.stop();
        if (limelight != null) {
            try { limelight.stop(); } catch (RuntimeException ignored) { }
        }
        dash.unregister();
    }

    private void cycle(boolean started) {
        double dt = Math.max(0.001, Math.min(0.2, loopTimer.seconds()));
        loopTimer.reset();
        double now = clock.seconds();

        handleGamepad(started);
        applyPipelineIfChanged();
        aim.setConfig(currentConfig());

        // ---- vision
        VisionFix fix = readVision(now);
        VisionAimController.Output out = aim.update(fix.visible, fix.txDeg, dt);

        // ---- turret output (always through the safety gate)
        double applied = 0;
        String outputStatus = turretFault.isEmpty() ? "" : turretFault;
        boolean deadman = started && gamepad1.right_bumper;
        if (io != null) {
            if (!started || !deadman || emergencyStop) io.stop();
            io.read(dt);
            io.command(out.demand, started && armed, deadman, emergencyStop);
            applied = io.appliedPower;
            outputStatus = io.safety.reason.isEmpty() ? "ENABLED" : io.safety.reason;
        }

        publish(started, fix, out, applied, outputStatus, deadman);
        dash.pushIfDirty();
    }

    // ================================================================ vision
    private static final class VisionFix {
        boolean visible;
        double txDeg = Double.NaN;
        double tyDeg = Double.NaN;
        int id = -1;
        String note = "";
        String allTags = "";
        double staleMs = Double.NaN;
        boolean resultPresent;
        AimPoint.Result point;
        double limelightTx = Double.NaN;
        double[] pose6;
    }

    private VisionFix readVision(double now) {
        VisionFix fix = new VisionFix();
        if (limelight == null) {
            fix.note = limelightFault;
            return fix;
        }
        LLResult r = limelight.getLatestResult();
        if (r == null) {
            fix.note = "No result from Limelight yet";
            return fix;
        }
        fix.resultPresent = true;
        fix.staleMs = r.getStaleness();

        double ts = r.getTimestamp();
        if (!Double.isNaN(ts) && ts != lastResultTimestamp) {
            lastResultTimestamp = ts;
            framesSeen++;
            if (!Double.isNaN(lastNewFrameSec) && now > lastNewFrameSec) {
                double inst = 1.0 / (now - lastNewFrameSec);
                fps += 0.1 * (inst - fps);
            }
            lastNewFrameSec = now;
        }

        if (r.getPipelineIndex() != (int) pipeline.v()) {
            fix.note = "Result is from pipeline " + r.getPipelineIndex() + ", wanted " + (int) pipeline.v();
            return fix;
        }
        if (!(fix.staleMs <= maxStaleMs.v())) {
            fix.note = String.format("Result stale (%.0f ms > %.0f ms)", fix.staleMs, maxStaleMs.v());
            return fix;
        }
        if (!r.isValid()) {
            fix.note = "No target in view";
            return fix;
        }

        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        StringBuilder sb = new StringBuilder();
        LLResultTypes.FiducialResult best = null;
        int wanted = (int) tagId.v();
        if (tags != null) {
            for (LLResultTypes.FiducialResult t : tags) {
                sb.append(String.format("#%d tx %.1f  ", t.getFiducialId(), t.getTargetXDegrees()));
                boolean match = wanted < 0 || t.getFiducialId() == wanted;
                if (match && (best == null
                        || Math.abs(t.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees()))) {
                    best = t;
                }
            }
        }
        fix.allTags = sb.toString();
        if (best != null) {
            fix.visible = true;
            fix.limelightTx = best.getTargetXDegrees();
            fix.txDeg = fix.limelightTx;
            fix.tyDeg = best.getTargetYDegrees();
            fix.id = best.getFiducialId();
            double[] pose6 = pose6(best);
            fix.pose6 = pose6;
            if (pose6 != null) {
                fix.point = AimPoint.compute(pose6, pointX.v(), pointY.v(), pointZ.v());
                if (aimAtPoint.v()) {
                    if (fix.point.valid) {
                        fix.txDeg = fix.point.yawDeg;
                    } else {
                        // Never silently fall back to the tag centre when the operator asked for a point.
                        fix.visible = false;
                        fix.note = "AIM_AT_POINT: " + fix.point.reason;
                    }
                }
            } else if (aimAtPoint.v()) {
                fix.visible = false;
                fix.note = "AIM_AT_POINT: tag has no 3D pose (is this the AprilTag pipeline with 3D enabled?)";
            }
            return fix;
        }
        if (useResultTx.v()) {
            fix.visible = true;
            fix.txDeg = r.getTx();
            fix.tyDeg = r.getTy();
            fix.note = "Using result tx (non-tag pipeline)";
            return fix;
        }
        fix.note = tags == null || tags.isEmpty()
                ? "Pipeline sees a target but no AprilTag. Is it the AprilTag pipeline?"
                : "Tags in view, but not ID " + wanted;
        return fix;
    }

    /** Limelight {@code [x, y, z, pitchDeg, yawDeg, rollDeg]} in camera space, inches and degrees. */
    private static double[] pose6(LLResultTypes.FiducialResult tag) {
        if (tag.getTargetPoseCameraSpace() == null) return null;
        Position position = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles o = tag.getTargetPoseCameraSpace().getOrientation();
        // SDK Pose3D stores YawPitchRollAngles(yaw=a[5], pitch=a[4], roll=a[3]); this restores the Limelight order.
        return new double[]{position.x, position.y, position.z,
                o.getRoll(AngleUnit.DEGREES), o.getPitch(AngleUnit.DEGREES), o.getYaw(AngleUnit.DEGREES)};
    }

    private void applyPipeline() {
        int wanted = (int) pipeline.v();
        if (limelight != null && wanted != appliedPipeline) {
            limelight.pipelineSwitch(wanted);
            appliedPipeline = wanted;
            aim.reset();
        }
    }

    private void applyPipelineIfChanged() { applyPipeline(); }

    private VisionAimController.Config currentConfig() {
        VisionAimController.Config c = new VisionAimController.Config();
        c.kP = Math.max(0, kP.v());
        c.kD = Math.max(0, kD.v());
        c.maxPower = maxPower.v();
        c.deadbandDeg = Math.max(0, deadband.v());
        c.lockDeg = Math.max(c.deadbandDeg, lockDeg.v());
        c.lockFrames = Math.max(1, (int) lockFrames.v());
        c.filterAlpha = filterAlpha.v();
        c.sign = aimSign.v() < 0 ? -1 : 1;
        c.aimOffsetDeg = aimOffset.v();
        return c;
    }

    // ================================================================ gamepad
    private void handleGamepad(boolean started) {
        boolean a = gamepad1.a, lb = gamepad1.left_bumper;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;

        if (started && gamepad1.b) {
            emergencyStop = true;
            armed = false;
        }
        if (started && gamepad1.back && gamepad1.start && !gamepad1.b && !gamepad1.right_bumper) {
            emergencyStop = false;
            armed = false;
            if (io != null) io.safety.clearFaults();
        }
        if (started && a && !pA && !emergencyStop) armed = !armed;
        if (lb && !pLb) aimSign.local(aimSign.v() < 0 ? 1 : -1);
        if (up && !pUp) kP.local(round(kP.v() + 0.002));
        if (down && !pDown) kP.local(round(Math.max(0, kP.v() - 0.002)));
        if (right && !pRight) kD.local(round(kD.v() + 0.0001));
        if (left && !pLeft) kD.local(round(Math.max(0, kD.v() - 0.0001)));
        pA = a; pLb = lb; pUp = up; pDown = down; pLeft = left; pRight = right;
    }

    private static double round(double v) { return Math.round(v * 1e6) / 1e6; }

    // ================================================================ telemetry
    private void publish(boolean started, VisionFix fix, VisionAimController.Output out,
                         double applied, String outputStatus, boolean deadman) {
        // Graph these in Dashboard: txDeg, filteredTxDeg, demand, appliedPower.
        multi.addData("txDeg", Double.isNaN(fix.txDeg) ? 0.0 : fix.txDeg);
        multi.addData("filteredTxDeg", Double.isNaN(out.filteredTxDeg) ? 0.0 : out.filteredTxDeg);
        multi.addData("demand", String.format("%.4f", out.demand));
        multi.addData("appliedPower", String.format("%.4f", applied));

        multi.addData("AIM STATE", out.state);
        multi.addData("Target", fix.visible ? String.format("tag #%d  tx %.2f  ty %.2f", fix.id, fix.txDeg, fix.tyDeg)
                : "NOT VISIBLE: " + fix.note);
        multi.addData("Locked", out.locked ? "YES" : "no");
        multi.addLine("--- AIM POINT (tag frame) ---");
        multi.addData("Aiming at", aimAtPoint.v() ? String.format("POINT (%.1f, %.1f, %.1f) in, tag frame", pointX.v(), pointY.v(), pointZ.v())
                : "TAG CENTRE (AIM_AT_POINT off)");
        if (fix.point != null && fix.point.valid) {
            multi.addData("Yaw: tag centre / aim point", "%.2f / %.2f deg  (Limelight tx %.2f)",
                    fix.point.tagYawDeg, fix.point.yawDeg, fix.limelightTx);
            multi.addData("Elevation: tag centre / aim point", "%.2f / %.2f deg", fix.point.tagElevationDeg, fix.point.elevationDeg);
            multi.addData("Tag pose in camera", "x %.1f y %.1f z %.1f in | pitch %.1f yaw %.1f roll %.1f deg",
                    fix.pose6[0], fix.pose6[1], fix.pose6[2], fix.pose6[3], fix.pose6[4], fix.pose6[5]);
            multi.addData("Range to aim point (horizontal / slant)", "%.1f / %.1f in", fix.point.horizontalRangeIn, fix.point.slantRangeIn);
        } else if (fix.point != null) {
            multi.addData("Aim point", "unavailable: " + fix.point.reason);
        } else {
            multi.addData("Aim point", "needs a visible tag with a 3D pose");
        }
        multi.addData("Error deg / rate deg/s", "%.2f / %.1f", out.errorDeg, out.errorRateDegPerSec);
        multi.addData("Demand (before safety gate)", "%.4f  (sign %+d)", out.demand, aimSign.v() < 0 ? -1 : 1);

        multi.addLine("--- TURRET OUTPUT ---");
        boolean blocked = Math.abs(out.demand) > 0 && applied == 0;
        multi.addData("Output", outputStatus.isEmpty() ? "n/a" : outputStatus);
        multi.addData("Turret commissioning", readiness.openLoop && readiness.closedLoop
                ? "ready for closed loop" : "NOT READY: " + readiness.missing);
        multi.addData("Armed / deadman (hold RB) / E-stop", "%s / %s / %s",
                armed ? "YES" : "no", deadman ? "YES" : "no", emergencyStop ? "LATCHED" : "clear");
        if (blocked) multi.addLine("DRY RUN: the vision demand is computed but the safety gate is blocking motion.");
        if (io != null) {
            multi.addData("Turret angle deg (encoder)", io.sample.valid ? String.format("%.2f", Math.toDegrees(io.getAngleRad()))
                    : "INVALID: " + io.sample.fault);
        }

        multi.addLine("--- LIMELIGHT ---");
        if (limelight != null) {
            multi.addData("Connected / running", "%s / %s", limelight.isConnected(), limelight.isRunning());
            multi.addData("Pipeline wanted / result", "%d / %s", (int) pipeline.v(),
                    fix.resultPresent ? "present" : "none");
            multi.addData("Result age ms", Double.isNaN(fix.staleMs) ? "n/a" : String.format("%.0f", fix.staleMs));
            multi.addData("Frames seen / fps", "%d / %.0f", framesSeen, fps);
        } else {
            multi.addData("Limelight", limelightFault);
        }
        multi.addData("All tags in view", fix.allTags.isEmpty() ? "none" : fix.allTags);

        multi.addLine("--- DASHBOARD LINK ---");
        multi.addData("Dashboard", dashStatus);
        multi.addData("Dashboard edits received", dash.editSummary());

        multi.addLine("--- TUNING (paste into VisionAimController.Config defaults when happy) ---");
        multi.addData("Gains", "kP %.4f  kD %.5f  max %.2f  deadband %.2f  sign %+d  offset %.2f",
                kP.v(), kD.v(), maxPower.v(), deadband.v(), aimSign.v() < 0 ? -1 : 1, aimOffset.v());
        multi.addLine("A arm | hold RB deadman | B e-stop | BACK+START clear | LB flip sign | D-pad U/D kP, L/R kD");
        multi.update();
    }
}
