package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.turret.TurretEncoderSample;

/** Owns four drive outputs for commissioning or a latched recovery mode. */
public final class RevBasicDrive {
    private final DcMotor[] motors=new DcMotor[4];
    private final double cap;
    private boolean fault;
    public RevBasicDrive(HardwareMap map,BasicDrive.Settings settings) {
        if(map==null||settings==null)throw new IllegalArgumentException("Measured basic drive required");
        cap=settings.powerCap;
        try {
            for(int i=0;i<4;i++) {
                motors[i]=map.get(DcMotor.class,settings.names[i]);motors[i].setPower(0);
                motors[i].setDirection(settings.reversed[i]?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }catch(RuntimeException failure){stop();throw failure;}
    }
    public boolean healthy() {return !fault;}
    public boolean write(double x,double y,double yaw,boolean enabled,double dt) {
        if(!enabled||!TurretEncoderSample.validDt(dt)||fault) {stop();return false;}
        double[] powers=BasicDrive.powers(x,y,yaw,cap);
        if(powers==null){fault=true;stop();return false;}
        try {for(int i=0;i<4;i++)motors[i].setPower(powers[i]);return true;}
        catch(RuntimeException failure){fault=true;stop();return false;}
    }
    public boolean stop() {
        for(DcMotor motor:motors)if(motor!=null)try {motor.setPower(0);}catch(RuntimeException failure){fault=true;}
        return !fault;
    }
}
