package org.firstinspires.ftc.teamcode.scoring;

import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Piece-specific measured knots. No extrapolation; supply only characterized interpolation intervals.
 * Intermediate hood/RPM combinations must be validated on the physical launcher.
 */
public final class CalibratedShotTable implements ShotSelector {
    public static final class Point {
        public final double rangeInches, flightSec, flightUncertaintySec;
        public final ShotRecipe recipe;
        public final double releaseDelaySec,releaseUncertaintySec,targetHeight,heightTolerance;
        public Point(double rangeInches, ShotRecipe recipe, double flightSec, double flightUncertaintySec) {
            this(rangeInches,recipe,flightSec,flightUncertaintySec,Double.NaN,Double.NaN,Double.NaN,Double.NaN);
        }
        public Point(double rangeInches,ShotRecipe recipe,double flightSec,double flightUncertaintySec,
                double releaseDelaySec,double releaseUncertaintySec,double targetHeight,double heightTolerance) {
            if (!Double.isFinite(rangeInches) || rangeInches <= 0 || recipe == null
                    || !recipe.rangeCalibrated || rangeInches < recipe.minRange || rangeInches > recipe.maxRange
                    || !new Selection(recipe, flightSec, flightUncertaintySec).hasFlightMeasurement())
                throw new IllegalArgumentException("Measured range, recipe and flight interval required");
            this.rangeInches = rangeInches; this.recipe = recipe;
            this.flightSec = flightSec; this.flightUncertaintySec = flightUncertaintySec;
            this.releaseDelaySec=releaseDelaySec;this.releaseUncertaintySec=releaseUncertaintySec;
            this.targetHeight=targetHeight;this.heightTolerance=heightTolerance;
            if(recipe.physical()&&(!new Selection(recipe,flightSec,flightUncertaintySec,releaseDelaySec,
                    releaseUncertaintySec,targetHeight,heightTolerance).hasReleaseMeasurement()
                    ||!Double.isFinite(targetHeight)||!Double.isFinite(heightTolerance)||heightTolerance<0))
                throw new IllegalArgumentException("Physical shots need measured release timing and target height");
        }
    }
    private final Map<BallType,List<Point>> tables = new EnumMap<BallType,List<Point>>(BallType.class);
    public CalibratedShotTable(List<Point> points) {
        if (points == null || points.isEmpty()) throw new IllegalArgumentException("Measured points required");
        for (Point p : points) {
            if (p == null) throw new IllegalArgumentException("Null calibration point");
            if (!tables.containsKey(p.recipe.type)) tables.put(p.recipe.type, new ArrayList<Point>());
            tables.get(p.recipe.type).add(p);
        }
        for (List<Point> table : tables.values()) {
            Collections.sort(table, (a,b) -> Double.compare(a.rangeInches,b.rangeInches));
            if (table.size() < 2) throw new IllegalArgumentException("At least two ranges per ball type required");
            for (int i=1;i<table.size();i++) {
                Point a=table.get(i-1),b=table.get(i);
                if(b.rangeInches<=a.rangeInches)throw new IllegalArgumentException("Duplicate calibration range");
                if(a.recipe.geometry!=b.recipe.geometry||a.recipe.physical()!=b.recipe.physical())
                    throw new IllegalArgumentException("One geometry revision per table required");
                if(a.recipe.physical()&&(a.recipe.gapMm!=b.recipe.gapMm||a.targetHeight!=b.targetHeight))
                    throw new IllegalArgumentException("Initial per-type tables hold gap and target height constant");
            }
        }
    }
    @Override public Selection select(BallType type, double range) {
        List<Point> table = type == null ? null : tables.get(type);
        if (table == null || !Double.isFinite(range) || range < table.get(0).rangeInches
                || range > table.get(table.size()-1).rangeInches) return null;
        for (int i=1;i<table.size();i++) {
            Point a=table.get(i-1), b=table.get(i);
            if (range > b.rangeInches) continue;
            double t=(range-a.rangeInches)/(b.rangeInches-a.rangeInches);
            ShotRecipe x=a.recipe,y=b.recipe;
            ShotRecipe recipe=new ShotRecipe(type,lerp(x.rpm,y.rpm,t),lerp(x.hood,y.hood,t),
                    lerp(x.compression,y.compression,t),Math.max(x.settleSec,y.settleSec),
                    Math.min(x.rpmTolerance,y.rpmTolerance),Math.max(x.rpmStableSec,y.rpmStableSec),
                    table.get(0).rangeInches,table.get(table.size()-1).rangeInches);
            if(x.physical())recipe=new ShotRecipe(type,lerp(x.rpm,y.rpm,t),lerp(x.pitchRad,y.pitchRad,t),x.gapMm,x.geometry,
                    Math.max(x.settleSec,y.settleSec),Math.min(x.rpmTolerance,y.rpmTolerance),Math.max(x.rpmStableSec,y.rpmStableSec),
                    table.get(0).rangeInches,table.get(table.size()-1).rangeInches);
            return new Selection(recipe,lerp(a.flightSec,b.flightSec,t),Math.max(a.flightUncertaintySec,b.flightUncertaintySec),
                    lerp(a.releaseDelaySec,b.releaseDelaySec,t),Math.max(a.releaseUncertaintySec,b.releaseUncertaintySec),
                    a.targetHeight,Math.min(a.heightTolerance,b.heightTolerance));
        }
        return null;
    }
    public List<Point> points() {
        List<Point> result=new ArrayList<Point>();
        for (List<Point> table:tables.values()) result.addAll(table);
        return Collections.unmodifiableList(result);
    }
    private static double lerp(double a,double b,double t) {return a+(b-a)*t;}
}
