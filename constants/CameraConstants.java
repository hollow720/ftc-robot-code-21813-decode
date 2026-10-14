package org.firstinspires.ftc.teamcode.constants;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.Map;

public class CameraConstants {
    public static Integer redTagID = 24;
    public static Integer blueTagID = 20;

    public static Map<Robot.Alliance, Integer> validTagMap = Map.of(Robot.Alliance.RED, redTagID, Robot.Alliance.BLUE, blueTagID);

    public static double staleSampleThreshold = 1000;

    public static double minConfidenceThreshold = 87.0;

    public static CameraConfig rightCamera =
            new CameraConfig(
                    361.3,
                    361.45,
                    472.64,
                    350.97,
                    new Size(960, 720),
                    new Size(1280, 960),
                    new Position(DistanceUnit.INCH, 7.6, 0.0, 6.38, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, -90.0, -70.0, 0.0, 0),
                    "rightCamera", 1, (long)20.0, 0);

    public static class CameraConfig
    {
        //Focal Length
        //In a perfect pinhole camera Fx == Fy
        final double Fx;
        final double Fy;
        //Center Point
        final double Cx;
        final double Cy;
        final Size tunedResolution;
        final Size requestedResolution;
        final String cameraName;

        final Position translation;
        final YawPitchRollAngles orientation;

        final int decimation;
        final long exposureMs;
        final int gain;

        public CameraConfig(double Fx, double Fy, double Cx, double Cy, Size tunedResolution, Size requestedResolution, Position translation, YawPitchRollAngles orientation, String cameraName, int decimation, long exposureMs, int gain)
        {
            this.Fx = Fx;
            this.Fy = Fy;
            this.Cx = Cx;
            this.Cy = Cy;

            this.tunedResolution = tunedResolution;
            this.requestedResolution = requestedResolution;
            this.cameraName = cameraName;

            this.translation = translation;
            this.orientation = orientation;

            this.decimation = decimation;
            this.exposureMs = exposureMs;
            this.gain = gain;
        }

        private boolean checkIfResolutionIsValid(Size requested)
        {
            if (tunedResolution.getWidth() * requested.getHeight() != requested.getWidth() * tunedResolution.getHeight())
            {
                throw new RuntimeException("Requested Resolution is invalid");
            }

            return true;
        }

        public double Fx()
        {
            checkIfResolutionIsValid(requestedResolution);
            return this.Fx * requestedResolution.getWidth()/this.tunedResolution.getWidth();
        }

        public double Fy()
        {
            checkIfResolutionIsValid(requestedResolution);
            return this.Fy * requestedResolution.getWidth()/this.tunedResolution.getWidth();
        }

        public double Cx()
        {
            checkIfResolutionIsValid(requestedResolution);
            return this.Cx * requestedResolution.getWidth()/this.tunedResolution.getWidth();
        }

        public double Cy()
        {
            checkIfResolutionIsValid(requestedResolution);
            return this.Cy * requestedResolution.getWidth()/this.tunedResolution.getWidth();
        }

        public String getCameraName()
        {
            return this.cameraName;
        }

        public Position getTranslation()
        {
            return this.translation;
        }

        public YawPitchRollAngles getOrientation()
        {
            return this.orientation;
        }

        public Size getResolution()
        {
            return this.requestedResolution;
        }

        public int getDecimation()
        {
            return this.decimation;
        }

        public long getExposureMs()
        {
            return this.exposureMs;
        }

        public int getGain()
        {
            return this.gain;
        }
    }
}
