package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Pieces already in the robot. Counts only. Order inside the robot does not
 * matter to the tip model.
 */
public final class BallLoad {

    private final int pollen;
    private final int nectar;

    public BallLoad(int pollen, int nectar) {
        this.pollen = Math.max(0, pollen);
        this.nectar = Math.max(0, nectar);
    }

    public static BallLoad empty() {
        return new BallLoad(0, 0);
    }

    public int getPollen() {
        return pollen;
    }

    public int getNectar() {
        return nectar;
    }

    public int getCount(BallType type) {
        if (type == BallType.NECTAR) {
            return nectar;
        }
        return pollen;
    }

    public int total() {
        return pollen + nectar;
    }

    public int remainingCapacity(int maxCapacity) {
        return Math.max(0, maxCapacity - total());
    }

    public BallLoad plus(BallType type) {
        if (type == BallType.NECTAR) {
            return new BallLoad(pollen, nectar + 1);
        }
        return new BallLoad(pollen + 1, nectar);
    }

    @Override
    public String toString() {
        return pollen + "P+" + nectar + "N";
    }
}
