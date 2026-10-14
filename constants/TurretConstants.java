package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.util.Util;

@Config
public class TurretConstants {
    public static Pose2d redGoal = new Pose2d(-62.0, 62.0, new Rotation2d());
    public static Pose2d blueGoal = new Pose2d(-62.0, -62.0, new Rotation2d());
//    public static double kP_normal_turret = 0.003;
//    public static double kS_turret = 0.075;
//    public static double setpointToleranceDegrees = 2;
//
//    public static double maxPower = 1.0;
//
//    public static boolean reversedQuadrature = false;

    public static Translation2d turretOffset = new Translation2d(
            Util.metersToInches(Util.millimetersToMeters(15.89309)),
            0
    );

    public static encoderConfig leftEncoderConfig
            = new encoderConfig(
            104.0,
            true,
            "leftEncoder",
            102.0,
            107.0
    );

    public static encoderConfig rightEncoderConfig
            = new encoderConfig(
            269.5,
            true,
            "rightEncoder",
            268.0,
            276.0
    );


    /**
     * The encoder is normalized for wiring so
     *
     * 0 and 360 are pointing left,
     * CCW
     */
    public static class encoderConfig
    {
        String name = "";
        boolean reversed = false;
        double zeroOffset = 0.0;

        double minBadAngle = 10;
        double maxBadAngle = 20;
        static double deadzoneOffset = 10;

        public encoderConfig(double zeroOffset, boolean reversed, String name, double minBadAngle, double maxBadAngle)
        {
            this.zeroOffset = zeroOffset;
            this.reversed = reversed;
            this.name = name;

            this.minBadAngle = minBadAngle;
            this.maxBadAngle = maxBadAngle;
        }

        public String getName()
        {
            return this.name;
        }

        /**
         * general purpose [0, 360] range cap
         */
        static double normalize360(double degrees) {
            return ((degrees % 360) + 360) % 360;
        }

        static double normalize180(double degrees) {
            double a = normalize360(degrees);
            return (a > 180) ? a - 360 : a;
        }

        public double normalize360Degrees(double degrees)
        {
            double zeroed = degrees - zeroOffset;
            return this.normalize360(reversed ? -zeroed : zeroed);
        }

        public double normalize180Degrees(double degrees)
        {
            double zeroed = degrees - zeroOffset;
            return this.normalize180(reversed ? -zeroed : zeroed);
        }


        public boolean nearDeadzone(double degrees)
        {
            return (degrees > (this.minBadAngle-deadzoneOffset)) && (degrees < (this.maxBadAngle+deadzoneOffset));
        }
    }
}
