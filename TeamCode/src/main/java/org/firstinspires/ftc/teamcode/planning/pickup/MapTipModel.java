package org.firstinspires.ftc.teamcode.planning.pickup;

import java.util.HashMap;
import java.util.Map;

/**
 * Tip table filled by the caller. Missing loads use {@code defaultProbability},
 * which is 0 unless one is supplied. This class does not contain game rates.
 */
public final class MapTipModel implements TipModel {

    private final double defaultProbability;
    private final boolean defaultKnown;
    private final Map<Long, Double> table = new HashMap<Long, Double>();

    public MapTipModel() {
        defaultProbability = 0.0;
        defaultKnown = false;
    }

    public MapTipModel(double defaultProbability) {
        requireProbability(defaultProbability);
        this.defaultProbability = clamp(defaultProbability);
        defaultKnown = Double.isFinite(defaultProbability);
    }

    @Override public boolean hasEstimate(BallLoad load) {
        return load != null && (defaultKnown || table.containsKey(key(load.getPollen(), load.getNectar())));
    }

    public MapTipModel set(int pollen, int nectar, double probability) {
        requireProbability(probability);
        if(pollen<0||nectar<0)throw new IllegalArgumentException("Nonnegative measured load required");
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
    private static void requireProbability(double probability) {
        if(!Double.isFinite(probability)||probability<0||probability>1)throw new IllegalArgumentException("Measured probability must be in [0,1]");
    }
}
