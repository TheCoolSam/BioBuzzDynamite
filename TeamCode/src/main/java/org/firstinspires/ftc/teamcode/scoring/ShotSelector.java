package org.firstinspires.ftc.teamcode.scoring;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Selects only characterized shots. Distances are inches and times are seconds. */
public interface ShotSelector {
    Selection select(BallType type, double rangeInches);

    final class Selection {
        public final ShotRecipe recipe;
        public final double flightSec, flightUncertaintySec;
        public final double releaseDelaySec,releaseUncertaintySec,targetHeight,heightTolerance;
        public Selection(ShotRecipe recipe, double flightSec, double flightUncertaintySec) {
            this(recipe,flightSec,flightUncertaintySec,Double.NaN,Double.NaN,Double.NaN,Double.NaN);
        }
        public Selection(ShotRecipe recipe,double flightSec,double flightUncertaintySec,double releaseDelaySec,
                double releaseUncertaintySec,double targetHeight,double heightTolerance) {
            if (recipe == null) throw new IllegalArgumentException("Recipe required");
            this.recipe = recipe;
            this.flightSec = flightSec;
            this.flightUncertaintySec = flightUncertaintySec;
            this.releaseDelaySec=releaseDelaySec;this.releaseUncertaintySec=releaseUncertaintySec;
            this.targetHeight=targetHeight;this.heightTolerance=heightTolerance;
        }
        public boolean hasReleaseMeasurement() {
            return Double.isFinite(releaseDelaySec)&&releaseDelaySec>0&&Double.isFinite(releaseUncertaintySec)
                    &&releaseUncertaintySec>0&&releaseUncertaintySec<releaseDelaySec;
        }
        public boolean acceptsHeight(double height) {
            return Double.isFinite(height)&&Double.isFinite(targetHeight)&&Double.isFinite(heightTolerance)
                    &&heightTolerance>=0&&Math.abs(height-targetHeight)<=heightTolerance;
        }
        public boolean hasFlightMeasurement() {
            return Double.isFinite(flightSec) && flightSec > 0
                    && Double.isFinite(flightUncertaintySec) && flightUncertaintySec >= 0
                    && flightUncertaintySec < flightSec;
        }
    }
}
