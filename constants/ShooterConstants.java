package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.sun.tools.javac.util.Pair;

import java.util.Map;
import java.util.TreeMap;

@Config
public class ShooterConstants {
    public static double retractedHoodPosition = 0.0;
    public static double fullExtensionHoodPosition = 0.52;

    public static double kV = 0.0002; //TODO: tune values
    public static double kS = 0.0829;

    public static double maxRPM = 5000;
    public static double RPMtolerance = 50;


    public static TreeMap<Double, Pair<Double, Double>> inchesToRPMandHood = new TreeMap<>();

    static {
        inchesToRPMandHood.put(40.0, Pair.of(2380.0, 0.05));
        inchesToRPMandHood.put(45.0, Pair.of(2440.0, 0.05));
        inchesToRPMandHood.put(50.0, Pair.of(2480.0, 0.15));
        inchesToRPMandHood.put(55.0, Pair.of(2550.0, 0.25));
        inchesToRPMandHood.put(60.0, Pair.of(2600.0, 0.31));
        inchesToRPMandHood.put(65.0, Pair.of(2670.0, 0.31));
        inchesToRPMandHood.put(70.0, Pair.of(2780.0, 0.33));
        inchesToRPMandHood.put(75.0, Pair.of(2840.0, 0.33));
        inchesToRPMandHood.put(80.0, Pair.of(2900.0, 0.36));
        inchesToRPMandHood.put(85.0, Pair.of(2940.0, 0.36));
        inchesToRPMandHood.put(90.0, Pair.of(3000.0, 0.36));
        inchesToRPMandHood.put(95.0, Pair.of(3050.0, 0.39));
        inchesToRPMandHood.put(100.0, Pair.of(3300.0, 0.47));
        inchesToRPMandHood.put(105.0, Pair.of(3350.0, 0.47));
        inchesToRPMandHood.put(110.0, Pair.of(3380.0, 0.44));
        inchesToRPMandHood.put(120.0, Pair.of(3500.0, 0.45));
        inchesToRPMandHood.put(125.0, Pair.of(3520.0, 0.45));
        inchesToRPMandHood.put(130.0, Pair.of(3570.0, 0.46));
        inchesToRPMandHood.put(135.0, Pair.of(3620.0, 0.43));
        inchesToRPMandHood.put(140.0, Pair.of(3680.0, 0.45));
    }

    public static TreeMap<Double, Double> inchesToShotTimeSeconds = new TreeMap<>();
    public static double hysteresisComp = 0;

    static {
        inchesToShotTimeSeconds.put(20.0, 0.0);
        inchesToShotTimeSeconds.put(40.0, 0.0);
        inchesToShotTimeSeconds.put(50.0, 0.0);
        inchesToShotTimeSeconds.put(70.0, 0.0);
        inchesToShotTimeSeconds.put(90.0, 0.0);
        inchesToShotTimeSeconds.put(100.0, 0.0);
    }

    public static Double getShotTime(double distanceFromGoalInches)
    {
        Map.Entry<Double, Double> ceiling = inchesToShotTimeSeconds.ceilingEntry(distanceFromGoalInches);
        Map.Entry<Double, Double> floor = inchesToShotTimeSeconds.floorEntry(distanceFromGoalInches);

        if (floor == null) {
            return inchesToShotTimeSeconds.firstEntry().getValue();
        } else if (ceiling == null) {
            return inchesToShotTimeSeconds.lastEntry().getValue();
        }

        double iTime =
                floor.getValue().doubleValue()
                        + (ceiling.getValue() - floor.getValue())
                                * (distanceFromGoalInches - floor.getKey())/(ceiling.getKey() - floor.getKey());
        return iTime;
    }

    /*
    first rpm second hood
     */
    public static Pair<Double, Double> interpolateFromShooterDist(double inches) {
        Map.Entry<Double, Pair<Double, Double>> ceiling = inchesToRPMandHood.ceilingEntry(inches);
        Map.Entry<Double, Pair<Double, Double>> floor = inchesToRPMandHood.floorEntry(inches);

        if (floor == null) {
            return inchesToRPMandHood.firstEntry().getValue();
        } else if (ceiling == null) {
            return inchesToRPMandHood.lastEntry().getValue();
        }

        Double iRPM = floor.getValue().fst + (ceiling.getValue().fst - floor.getValue().fst) * (inches - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        Double iHood = floor.getValue().snd + (ceiling.getValue().snd - floor.getValue().snd) * (inches - floor.getKey()) / (ceiling.getKey() - floor.getKey());

        return new Pair<>(iRPM, iHood);
    }
}
