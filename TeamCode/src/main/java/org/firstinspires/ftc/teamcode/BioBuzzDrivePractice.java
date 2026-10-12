package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.robot.*;

/** Four-motor commissioning/practice mode. No launcher, vision, odometry or path tune required. */
@TeleOp(name="BIOBUZZ Drive Practice",group="Commissioning")
public final class BioBuzzDrivePractice extends OpMode {
    private RevBasicDrive drive;
    private long last;
    private String reason="Supply DriveHardwareConfig.measured() after signs and stop are verified";
    @Override public void init() {
        BasicDrive.Settings settings=DriveHardwareConfig.measured();
        if(settings!=null)try {drive=new RevBasicDrive(hardwareMap,settings);reason="ROBOT CENTRIC / HOLD RB";}
        catch(RuntimeException failure){reason=failure.getMessage();}
        telemetry.addData("Drive",reason);
    }
    @Override public void start() {last=System.nanoTime();if(drive!=null)drive.stop();}
    @Override public void loop() {
        long now=System.nanoTime();double dt=(now-last)*1e-9;last=now;
        if(drive!=null)drive.write(deadband(gamepad1.left_stick_x),deadband(-gamepad1.left_stick_y),
                deadband(-gamepad1.right_stick_x),gamepad1.right_bumper&&!gamepad1.b,dt);
        telemetry.addData("Drive",drive!=null&&!drive.healthy()?"WRITE FAULT / REINITIALIZE":reason);
        telemetry.addLine("RB enable; B stop; left stick translates; right stick turns. Mechanisms are not mapped.");
    }
    private static double deadband(double n) {return Math.abs(n)<.08?0:n;}
    @Override public void stop() {if(drive!=null)drive.stop();}
}
