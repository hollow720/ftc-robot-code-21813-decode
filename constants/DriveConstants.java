package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.control.kinematics.SixWheelMecanumKinematics;
import org.firstinspires.ftc.teamcode.util.Util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

@Config
public class DriveConstants {
    //NWU coordinate plate btw
    //millimeters
    public static double xWheelOffset = Util.millimetersToMeters(135.0);
    //millimeters
    public static double yWheelOffset = Util.millimetersToMeters(196.425);
    //millimeters
    public static double wheelDiameter = Util.millimetersToMeters(104.0);

    //Vector R from the center of the robot
    public static double shiftedXMeters = 0; //0.05
    public static double shiftedYMeters = 0.0;
    public static Translation2d shiftedCOR = new Translation2d(shiftedXMeters, shiftedYMeters);
    public static Translation2d frontLeftR = new Translation2d(xWheelOffset/2.0, yWheelOffset/2.0).plus(shiftedCOR);
    public static Translation2d frontRightR = new Translation2d(xWheelOffset/2.0, -yWheelOffset/2.0).plus(shiftedCOR);
    public static Translation2d middleLeftR = new Translation2d(0/2.0, yWheelOffset/2.0).plus(shiftedCOR);
    public static Translation2d middleRightR = new Translation2d(0/2.0, -yWheelOffset/2.0).plus(shiftedCOR);
    public static Translation2d backLeftR = new Translation2d(-xWheelOffset/2.0, yWheelOffset/2.0).plus(shiftedCOR);
    public static Translation2d backRightR = new Translation2d(-xWheelOffset/2.0, -yWheelOffset/2.0).plus(shiftedCOR);


    public static SixWheelMecanumKinematics kinematics = new SixWheelMecanumKinematics(
            frontLeftR,
            middleLeftR,
            backLeftR,
            frontRightR,
            middleRightR,
            backRightR);

    public static String frontRightName = "frontRight";
    public static String frontLeftName = "frontLeft";
    public static String middleRightName = "middleRight";
    public static String middleLeftName = "middleLeft";
    public static String backRightName = "backRight";
    public static String backLeftName = "backLeft";


    // ---  chunk 2 ---
    public static double m_maxTranslationalVelocityMetersPerSecond = 3.27; // Same thing except idk why this like this
    public static final DoubleSupplier maxTranslationalVelocityMetersPerSecond = () -> m_maxTranslationalVelocityMetersPerSecond;

    // ---  chunk 1 ---
    public static double m_maxWheelAngularVelocityRadiansPerSecond = m_maxTranslationalVelocityMetersPerSecond/(wheelDiameter/2.0); //this was just wrong because motors run at different RPM under load
    public static final DoubleSupplier maxWheelAngularVelocityRadiansPerSecond = () -> m_maxWheelAngularVelocityRadiansPerSecond;

    // ---  chunk 2 ---
    public static double m_maxRotationalVelocityRadiansPerSecond = 9;
//    public static double m_maxRotationalVelocityRadiansPerSecond = maxTranslationalVelocityMetersPerSecond.getAsDouble() / (Math.sqrt(Math.pow(frontLeftR.getY(), 2) + Math.pow(Math.abs(frontLeftR.getX()), 2)));
    //    public static volatile double m_maxRotationalVelocityRadiansPerSecond = 3.0; // I remember that the reason why i overcompensated is for curve following or smth
    public static final DoubleSupplier maxRotationalVelocityRadiansPerSecond = () -> m_maxRotationalVelocityRadiansPerSecond;

    public static double strafeRotCompensation = 0;

    @Config
    public static class ModelConstants
    {
        public static double kV = 0.4546;
        public static double kS = 0.0488;
    }
}
