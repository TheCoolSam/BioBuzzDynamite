package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotHardwareConfig;

/** Calibrated stationary scoring, with no moving-shot or burst dependency. */
@TeleOp(name="BIOBUZZ Stationary Scoring",group="Competition")
public final class BioBuzzStationaryTeleOp extends BioBuzzTeleOp {
    @Override protected RobotHardwareConfig.Settings loadSettings() {return RobotHardwareConfig.stationaryMeasured();}
    @Override protected boolean stationaryProfileRequired() {return true;}
}
