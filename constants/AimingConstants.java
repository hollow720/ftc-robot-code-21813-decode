package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AimingConstants {
    public static double translationalKp_x = 0.2;
    public static double translationalKp_y = 0.016;
    public static double closeTranslationalKp_x = 0.1;
    public static double closeTranslationalKp_y = 0.03;
    public static double rotationalKp = 8.0;
    public static double translationalProfiledKp = 0.1;
    public static double rotationalProfiledKp = 0.2;
    public static double maxVx = DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble(); // * 1.1
    public static double maxVy = DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble();
    public static double maxAxy = 3.6; //  m/s^2

    public static double maxVw = DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble();
    public static double maxAw = 150; // rad/s^2

    public static double feedforwardInertiaRot = 0;

    public static double translationToleranceRegular = 3;
    public static double rotationToleranceRegularRads = Math.toRadians(3);
    public static double translationToleranceProfiled = translationToleranceRegular;
    public static double rotationToleranceProfiledRads = rotationToleranceRegularRads;
    public static double powerfulTranslationClamp = 0.2;
    public static double powerfulRotationClamp = 0.2;
}
