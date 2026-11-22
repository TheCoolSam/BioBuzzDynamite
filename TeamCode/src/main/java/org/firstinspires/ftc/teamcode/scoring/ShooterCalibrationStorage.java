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
 *
 * <p>Schema history:
 * <ul>
 *   <li>1 - original bench tuner. Its PIDF/RPM numbers were tuned on a wrong scale (P larger than F)
 *       and are deliberately IGNORED on load. Hood/compression values are kept.</li>
 *   <li>2 - current. PIDF, RPM, open-loop power and ticks-per-rev are trusted.</li>
 * </ul>
 */
public final class ShooterCalibrationStorage {
    public static final String FILE_NAME = "biobuzz_shooter_calibration.properties";
    // 3: speed loop moved to software with battery compensation; old saved P/I (P 0.1 * F, I 0) are discarded.
    public static final int SCHEMA = 3;

    public static class Data {
        public double pollenCompression = 0.30;
        public double nectarCompression = 0.50;
        public double hoodPosition = 0.35;
        public double targetRpm = 3000.0;
        public double openLoopPower = 0.90; // the power the team ran the 6000 rpm flywheel at in OnBot Java
        public double ticksPerRev = 28.0;
        // Starting point only (software speed loop, hub units): F ~= 32767 * power / tps at 12 V,
        // P and I ~= 1.2 * F. Measure F on the bench.
        public double p = 15.0;
        public double i = 15.0;
        public double d = 0.0;
        public double f = 12.5;
        /** True when the file on disk predates {@link #SCHEMA} and its PIDF/RPM numbers were discarded. */
        public boolean legacyPidfDiscarded = false;
    }

    public static Data load() {
        Data data = new Data();
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (file.exists()) {
                String text = ReadWriteFile.readFile(file);
                Properties props = new Properties();
                props.load(new StringReader(text));
                int schema = (int) Double.parseDouble(props.getProperty("schema", "1"));
                data.pollenCompression = read(props, "pollenCompression", data.pollenCompression);
                data.nectarCompression = read(props, "nectarCompression", data.nectarCompression);
                data.hoodPosition = read(props, "hoodPosition", data.hoodPosition);
                if (schema >= 2) {
                    data.targetRpm = read(props, "targetRpm", data.targetRpm);
                    data.openLoopPower = read(props, "openLoopPower", data.openLoopPower);
                    data.ticksPerRev = read(props, "ticksPerRev", data.ticksPerRev);
                    if (schema >= 3) {
                        data.p = read(props, "p", data.p);
                        data.i = read(props, "i", data.i);
                    } else {
                        data.legacyPidfDiscarded = true;   // F still valid; P/I were sized for the hub loop
                    }
                    data.d = read(props, "d", data.d);
                    data.f = read(props, "f", data.f);
                } else {
                    data.legacyPidfDiscarded = true;
                }
                RobotLog.ii("ShooterCalibration", "Loaded calibration (schema " + schema + ") from " + FILE_NAME);
            }
        } catch (Exception e) {
            RobotLog.ee("ShooterCalibration", "Failed to load calibration: " + e.getMessage());
        }
        return data;
    }

    private static double read(Properties props, String key, double fallback) {
        double value = Double.parseDouble(props.getProperty(key, Double.toString(fallback)));
        return Double.isFinite(value) ? value : fallback;
    }

    public static boolean save(Data data) {
        if (data == null) return false;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            Properties props = new Properties();
            props.setProperty("schema", Integer.toString(SCHEMA));
            props.setProperty("pollenCompression", Double.toString(data.pollenCompression));
            props.setProperty("nectarCompression", Double.toString(data.nectarCompression));
            props.setProperty("hoodPosition", Double.toString(data.hoodPosition));
            props.setProperty("targetRpm", Double.toString(data.targetRpm));
            props.setProperty("openLoopPower", Double.toString(data.openLoopPower));
            props.setProperty("ticksPerRev", Double.toString(data.ticksPerRev));
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
