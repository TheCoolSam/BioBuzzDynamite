package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

/** Commissioning profile. Configured for four-motor drivebase practice. */
public final class DriveHardwareConfig {
    public static BasicDrive.Settings measured() {
        return new BasicDrive.Settings(
            new String[]{
                PedroConstants.FRONT_LEFT_MOTOR,
                PedroConstants.BACK_LEFT_MOTOR,
                PedroConstants.FRONT_RIGHT_MOTOR,
                PedroConstants.BACK_RIGHT_MOTOR
            },
            new boolean[]{false, false, false, false},
            0.6,
            true
        );
    }
    private DriveHardwareConfig() { }
}
