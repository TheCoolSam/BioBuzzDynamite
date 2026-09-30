package org.firstinspires.ftc.teamcode.planning.pickup;

import java.util.HashMap;
import java.util.Map;

/**
 * Tip table filled by the caller. Missing loads use {@code defaultProbability},
 * which is 0 unless one is supplied. This class does not contain game rates.
 */
public final class MapTipModel implements TipModel {

    private final double defaultProbability;
    private final Map<Long, Double> table = new HashMap<Long, Double>();

    public MapTipModel() {
        this(0.0);
    }

    public MapTipModel(double defaultProbability) {
        this.defaultProbability = clamp(defaultProbability);
    }

    public MapTipModel set(int pollen, int nectar, double probability) {
        table.put(key(pollen, nectar), Double.valueOf(clamp(probability)));
        return this;
    }

    @Override
    public double getTipProbability(BallLoad load) {
        if (load == null) {
            return defaultProbability;
        }
        Double stored = table.get(key(load.getPollen(), load.getNectar()));
        return stored == null ? defaultProbability : stored.doubleValue();
    }

    private static long key(int pollen, int nectar) {
        return (((long) pollen) << 32) | (nectar & 0xffffffffL);
    }

    private static double clamp(double probability) {
        if (!Double.isFinite(probability) || probability <= 0.0) {
            return 0.0;
        }
        if (probability >= 1.0) {
            return 1.0;
        }
        return probability;
    }
}
