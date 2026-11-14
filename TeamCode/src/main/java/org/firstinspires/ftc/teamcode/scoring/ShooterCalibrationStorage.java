package org.firstinspires.ftc.teamcode.scoring;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.Properties;

/**
 * Persistent calibration storage for shooter compression, hood angle, RPM, and flywheel PIDF.
 * Saves values to Control Hub disk so tuned parameters persist across power cycles and OpModes.
 */
public final class ShooterCalibrationStorage {
    public static final String FILE_NAME = "biobuzz_shooter_calibration.properties";

    public static class Data {
        public double pollenCompression = 0.30;
        public double nectarCompression = 0.50;
        public double hoodPosition = 0.35;
        public double targetRpm = 1400.0;
        public double p = 15.0;
        public double i = 0.0;
        public double d = 0.5;
        public double f = 12.5;
    }

    public static Data load() {
        Data data = new Data();
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (file.exists()) {
                String text = ReadWriteFile.readFile(file);
                Properties props = new Properties();
                props.load(new StringReader(text));
                data.pollenCompression = Double.parseDouble(props.getProperty("pollenCompression", Double.toString(data.pollenCompression)));
                data.nectarCompression = Double.parseDouble(props.getProperty("nectarCompression", Double.toString(data.nectarCompression)));
                data.hoodPosition = Double.parseDouble(props.getProperty("hoodPosition", Double.toString(data.hoodPosition)));
                data.targetRpm = Double.parseDouble(props.getProperty("targetRpm", Double.toString(data.targetRpm)));
                data.p = Double.parseDouble(props.getProperty("p", Double.toString(data.p)));
                data.i = Double.parseDouble(props.getProperty("i", Double.toString(data.i)));
                data.d = Double.parseDouble(props.getProperty("d", Double.toString(data.d)));
                data.f = Double.parseDouble(props.getProperty("f", Double.toString(data.f)));
                RobotLog.ii("ShooterCalibration", "Loaded calibration from " + FILE_NAME);
            }
        } catch (Exception e) {
            RobotLog.ee("ShooterCalibration", "Failed to load calibration: " + e.getMessage());
        }
        return data;
    }

    public static boolean save(Data data) {
        if (data == null) return false;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            Properties props = new Properties();
            props.setProperty("pollenCompression", Double.toString(data.pollenCompression));
            props.setProperty("nectarCompression", Double.toString(data.nectarCompression));
            props.setProperty("hoodPosition", Double.toString(data.hoodPosition));
            props.setProperty("targetRpm", Double.toString(data.targetRpm));
            props.setProperty("p", Double.toString(data.p));
            props.setProperty("i", Double.toString(data.i));
            props.setProperty("d", Double.toString(data.d));
            props.setProperty("f", Double.toString(data.f));
            StringWriter writer = new StringWriter();
            props.store(writer, "BioBuzz Shooter Calibration - Compression, Hood, RPM & PIDF");
            ReadWriteFile.writeFile(file, writer.toString());
            RobotLog.ii("ShooterCalibration", "Saved calibration to " + FILE_NAME);
            return true;
        } catch (Exception e) {
            RobotLog.ee("ShooterCalibration", "Failed to save calibration: " + e.getMessage());
            return false;
        }
    }
}
