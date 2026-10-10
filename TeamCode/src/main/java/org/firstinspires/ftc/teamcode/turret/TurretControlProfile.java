package org.firstinspires.ftc.teamcode.turret;

/** Inject measured mechanism dynamics; the default profile is for existing simulations only. */
public final class TurretControlProfile {
    public final double PHYSICAL_MIN_RAD,PHYSICAL_MAX_RAD,OPERATING_MIN_RAD,OPERATING_MAX_RAD;
    public final double UNWIND_ENTER_RAD,UNWIND_EXIT_RAD,UNWIND_KP,MAX_REQUESTED_CHASSIS_OMEGA;
    public final double UNWIND_OMEGA_RAMP_RAD_PER_SEC2,KP,KD,KV,MIN_MOTOR_POWER,MAX_MOTOR_POWER;
    public final double maxSpeed,maxAcceleration;
    public final boolean measured;
    public TurretControlProfile(double physicalMin,double physicalMax,double operatingMin,double operatingMax,
            double enter,double exit,double unwindKp,double omegaCap,double omegaRamp,
            double kp,double kd,double kv,double powerCap,double maxSpeed,double maxAcceleration) {
        this(physicalMin,physicalMax,operatingMin,operatingMax,enter,exit,unwindKp,omegaCap,omegaRamp,
                kp,kd,kv,powerCap,maxSpeed,maxAcceleration,true);
    }
    private TurretControlProfile(double physicalMin,double physicalMax,double operatingMin,double operatingMax,
            double enter,double exit,double unwindKp,double omegaCap,double omegaRamp,
            double kp,double kd,double kv,double powerCap,double maxSpeed,double maxAcceleration,boolean measured) {
        for (double n:new double[]{physicalMin,physicalMax,operatingMin,operatingMax,enter,exit,unwindKp,
                omegaCap,omegaRamp,kp,kd,kv,powerCap,maxSpeed,maxAcceleration}) if (!Double.isFinite(n)) throw new IllegalArgumentException("Finite measured profile required");
        if (!(physicalMin<operatingMin&&operatingMin<0&&operatingMax>0&&operatingMax<physicalMax
                &&physicalMin>-Math.PI&&physicalMax<Math.PI&&exit>0&&enter>exit
                &&enter<Math.min(-operatingMin,operatingMax)&&unwindKp>0&&omegaCap>0&&omegaRamp>0
                &&kp>0&&kd>=0&&kv>=0&&powerCap>0&&powerCap<=1&&maxSpeed>0&&maxAcceleration>0)) throw new IllegalArgumentException("Ordered limits and positive dynamics required");
        PHYSICAL_MIN_RAD=physicalMin;PHYSICAL_MAX_RAD=physicalMax;OPERATING_MIN_RAD=operatingMin;OPERATING_MAX_RAD=operatingMax;
        UNWIND_ENTER_RAD=enter;UNWIND_EXIT_RAD=exit;UNWIND_KP=unwindKp;MAX_REQUESTED_CHASSIS_OMEGA=omegaCap;
        UNWIND_OMEGA_RAMP_RAD_PER_SEC2=omegaRamp;KP=kp;KD=kd;KV=kv;MIN_MOTOR_POWER=-powerCap;MAX_MOTOR_POWER=powerCap;
        this.maxSpeed=maxSpeed;this.maxAcceleration=maxAcceleration;this.measured=measured;
    }
    public static TurretControlProfile simulation() {
        return new TurretControlProfile(TurretConstants.PHYSICAL_MIN_RAD,TurretConstants.PHYSICAL_MAX_RAD,
                TurretConstants.OPERATING_MIN_RAD,TurretConstants.OPERATING_MAX_RAD,TurretConstants.UNWIND_ENTER_RAD,
                TurretConstants.UNWIND_EXIT_RAD,TurretConstants.UNWIND_KP,TurretConstants.MAX_REQUESTED_CHASSIS_OMEGA,
                TurretConstants.UNWIND_OMEGA_RAMP_RAD_PER_SEC2,TurretConstants.KP,TurretConstants.KD,TurretConstants.KV,
                TurretConstants.MAX_MOTOR_POWER,1,1,false);
    }
}
