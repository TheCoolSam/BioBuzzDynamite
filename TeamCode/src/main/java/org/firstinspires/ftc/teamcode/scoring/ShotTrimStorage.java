package org.firstinspires.ftc.teamcode.scoring;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.Properties;

/** Saves driver 2's {@link ShotTrim} on the Control Hub so corrections carry from match to match. */
public final class ShotTrimStorage {
    public static final String FILE_NAME = "biobuzz_shot_trim.properties";

    private ShotTrimStorage() { }

    public static ShotTrim load() {
        ShotTrim trim = new ShotTrim();
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return trim;
            Properties props = new Properties();
            props.load(new StringReader(ReadWriteFile.readFile(file)));
            trim.restore(read(props, "rpmNear"), read(props, "rpmMid"), read(props, "rpmFar"), read(props, "aimRad"));
            RobotLog.ii("ShotTrim", "Loaded " + trim.describe());
        } catch (Exception e) {
            RobotLog.ee("ShotTrim", "Failed to load trims: " + e.getMessage());
        }
        return trim;
    }

    public static boolean save(ShotTrim trim) {
        if (trim == null) return false;
        try {
            Properties props = new Properties();
            props.setProperty("rpmNear", Double.toString(trim.rpmFraction(0)));
            props.setProperty("rpmMid", Double.toString(trim.rpmFraction(1)));
            props.setProperty("rpmFar", Double.toString(trim.rpmFraction(2)));
            props.setProperty("aimRad", Double.toString(trim.aimRad()));
            StringWriter writer = new StringWriter();
            props.store(writer, "BioBuzz driver-2 shot trims (fractions of calibrated rpm; aim in radians)");
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(FILE_NAME), writer.toString());
            return true;
        } catch (Exception e) {
            RobotLog.ee("ShotTrim", "Failed to save trims: " + e.getMessage());
            return false;
        }
    }

    private static double read(Properties props, String key) {
        try { return Double.parseDouble(props.getProperty(key, "0")); }
        catch (NumberFormatException e) { return 0; }
    }
}
