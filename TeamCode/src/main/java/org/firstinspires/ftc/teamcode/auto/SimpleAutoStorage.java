package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Properties;

/**
 * Saves the Simple Auto's tuned values to the Control Hub so they survive INIT, STOP and power cycles.
 * Dashboard edits alone reset to the code defaults every time the OpMode is initialised.
 *
 * <p>Only auto-specific values (route, distances, speeds, feed timing) live here. Shooter values
 * (hood, compression, power, rpm, PIDF) always come from the bench tuner's file
 * ({@code ShooterCalibrationStorage}), so tuning the shooter on the bench carries over automatically.
 */
public final class SimpleAutoStorage {
    public static final String FILE_NAME = "biobuzz_simple_auto.properties";
    public static final int SCHEMA = 1;

    private SimpleAutoStorage() { }

    public static Map<String, String> load() {
        Map<String, String> out = new LinkedHashMap<String, String>();
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return out;
            Properties props = new Properties();
            props.load(new StringReader(ReadWriteFile.readFile(file)));
            for (String key : props.stringPropertyNames()) out.put(key, props.getProperty(key));
            RobotLog.ii("SimpleAuto", "Loaded " + out.size() + " values from " + FILE_NAME);
        } catch (Exception e) {
            RobotLog.ee("SimpleAuto", "Failed to load " + FILE_NAME + ": " + e.getMessage());
        }
        return out;
    }

    public static boolean save(Map<String, String> values) {
        if (values == null) return false;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            Properties props = new Properties();
            props.setProperty("schema", Integer.toString(SCHEMA));
            for (Map.Entry<String, String> e : values.entrySet()) props.setProperty(e.getKey(), e.getValue());
            StringWriter writer = new StringWriter();
            props.store(writer, "BioBuzz Simple Auto tuning");
            ReadWriteFile.writeFile(file, writer.toString());
            RobotLog.ii("SimpleAuto", "Saved " + values.size() + " values to " + FILE_NAME);
            return true;
        } catch (Exception e) {
            RobotLog.ee("SimpleAuto", "Failed to save " + FILE_NAME + ": " + e.getMessage());
            return false;
        }
    }
}
