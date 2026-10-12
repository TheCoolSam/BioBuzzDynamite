package org.firstinspires.ftc.teamcode.scoring;

import java.io.*;
import java.util.*;
import org.firstinspires.ftc.teamcode.planning.pickup.BallType;

/** Strict numeric CSV import. Separate measured files for the two ball types, no default data. */
public final class ShotCalibrationCsv {
    public static final String HEADER="rangeInches,rpm,pitchDeg,gapMm,settleSec,rpmTolerance,rpmStableSec,flightSec,flightUncertaintySec,releaseDelaySec,releaseUncertaintySec,targetHeightInches,heightToleranceInches";
    public static CalibratedShotTable load(Reader pollen,Reader nectar,LauncherGeometry geometry) throws IOException {
        List<CalibratedShotTable.Point> points=new ArrayList<CalibratedShotTable.Point>();
        read(pollen,BallType.POLLEN,geometry,points);read(nectar,BallType.NECTAR,geometry,points);
        return new CalibratedShotTable(points);
    }
    private static void read(Reader reader,BallType type,LauncherGeometry geometry,List<CalibratedShotTable.Point> points) throws IOException {
        if(reader==null||geometry==null)throw new IllegalArgumentException("Both measured tables and geometry required");
        BufferedReader input=new BufferedReader(reader);
        if(!HEADER.equals(input.readLine()))throw new IllegalArgumentException("Shot CSV header/units mismatch: "+type);
        List<double[]> rows=new ArrayList<double[]>();
        double min=Double.POSITIVE_INFINITY,max=Double.NEGATIVE_INFINITY;
        String line;int number=1;
        while((line=input.readLine())!=null) {
            number++;if(line.trim().isEmpty())continue;
            String[] cells=line.split(",",-1);if(cells.length!=13)throw new IllegalArgumentException("Invalid "+type+" row "+number);
            double[] values=new double[13];
            for(int i=0;i<13;i++) {
                values[i]=Double.parseDouble(cells[i].trim());
                if(!Double.isFinite(values[i]))throw new IllegalArgumentException("Unmeasured "+type+" row "+number);
            }
            rows.add(values);min=Math.min(min,values[0]);max=Math.max(max,values[0]);
        }
        if(rows.size()<2)throw new IllegalArgumentException("Measure multiple distances for "+type);
        for(double[] v:rows) {
            ShotRecipe r=new ShotRecipe(type,v[1],Math.toRadians(v[2]),v[3],geometry,v[4],v[5],v[6],min,max);
            points.add(new CalibratedShotTable.Point(v[0],r,v[7],v[8],v[9],v[10],v[11],v[12]));
        }
    }
    private ShotCalibrationCsv() { }
}
