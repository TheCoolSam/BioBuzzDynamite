package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Selects only characterized shots. Distances are inches and times are seconds. */
public interface ShotSelector {
    Selection select(BallType type, double rangeInches);

    final class Selection {
        public final ShotRecipe recipe;
        public final double flightSec, flightUncertaintySec;
        public Selection(ShotRecipe recipe, double flightSec, double flightUncertaintySec) {
            if (recipe == null) throw new IllegalArgumentException("Recipe required");
            this.recipe = recipe;
            this.flightSec = flightSec;
            this.flightUncertaintySec = flightUncertaintySec;
        }
        public boolean hasFlightMeasurement() {
            return Double.isFinite(flightSec) && flightSec > 0
                    && Double.isFinite(flightUncertaintySec) && flightUncertaintySec >= 0
                    && flightUncertaintySec < flightSec;
        }
    }
}
