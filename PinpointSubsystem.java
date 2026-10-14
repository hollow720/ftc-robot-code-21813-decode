package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpoint;

@Config
public class PinpointSubsystem extends TelemetrySubsystem {
    GoBildaPinpointDriver pinpoint;
    static Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Pose2D lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver.DeviceStatus deviceStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY; //Notified on change

    public static double xEncOffset = -144.0;
    public static double yEncOffset = -39.0;
    public PinpointSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

//        pinpoint.initialize();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(xEncOffset, yEncOffset, DistanceUnit.MM);
        resetPinpointDisconnect(telemetry);
        pinpoint.setPosition(pose);

        deviceStatus = pinpoint.getDeviceStatus();
    }

    public void resetPinpointDisconnect(Telemetry telemetry)
    {
        double waitTime = 700;
//        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
        pinpoint.update();
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() < waitTime && pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
        {
            telemetryC.accept("Recalibrating POSE and IMU. Status: ", pinpoint.getDeviceStatus());
            telemetryC.accept("Timer: ", String.format("%f/%f", timer.milliseconds(), waitTime));
            telemetry.update();
        }
        telemetryC.accept("POSE and IMU Recalibrated. Status: ", pinpoint.getDeviceStatus());
        pinpoint.update();

    }

    @Override
    public void periodic() {
        pinpoint.update();
        if (Double.isNaN(pinpoint.getPosX(DistanceUnit.INCH)) || Double.isNaN(pinpoint.getPosY(DistanceUnit.INCH)))
        {
            pose = lastGoodPose;
            Log.i("%11", "pinpoint NaN value");
        }
        else{
            pose = pinpoint.getPosition();
            lastGoodPose = pose;
        }

        if (pinpoint.getDeviceStatus() != deviceStatus){
            Log.i("%Pinpoint Status Change", pinpoint.getDeviceStatus().toString());
            deviceStatus = pinpoint.getDeviceStatus();
        }

        telemetryC.accept("Pinpoint Velo X (Field)", getVelocityAmerican().getX());
        telemetryC.accept("Pinpoint Velo Y (Field)", getVelocityAmerican().getY());
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus()
    {
        return pinpoint.getDeviceStatus();
    }

    /*
    Returns SDK Pose
    This is in Inches and Degrees
     */
    private Pose2D getSDKPose() {
        return pose;
    }

    /*
    Returns FTCLib Pose2d object
    This is composed of a Translation2d and a Rotation2d object
    The Translation2d is in inches
    The Rotation2d can be radians or inches, but is CCW
     */
    public Pose2d getPose() {
        return new Pose2d(getSDKPose().getX(DistanceUnit.INCH),
                getSDKPose().getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.DEGREES)));
    }

    public Pose2d getFieldRelativeVelocityMetric() {
        return new Pose2d(pinpoint.getVelX(DistanceUnit.METER),
                pinpoint.getVelY(DistanceUnit.METER),
                new Rotation2d(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)));
    }

    public Pose2d getVelocityAmerican() {
        return new Pose2d(pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                new Rotation2d(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
    }

    /**
     * X - meters/s
     * Y - meters/s
     * Rotation2d rad/s
     * @return
     */
//    public Pose2d getFieldRelativeVelocityMetric()
//    {
//        Translation2d robotRelativeVelocity =
//                new Translation2d(
//                        pinpoint.getVelX(DistanceUnit.METER),
//                        pinpoint.getVelY(DistanceUnit.METER)
//                );
//
//        Translation2d fieldRelativeVelocity =
//                robotRelativeVelocity.rotateBy(
//                    new Rotation2d(
//                            pinpoint.getHeading(AngleUnit.RADIANS)
//                    ).unaryMinus()
//        );
//
//        return new Pose2d(
//                fieldRelativeVelocity.getX(),
//                fieldRelativeVelocity.getY(),
//                new Rotation2d(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
//        );
//    }
//
    /**
     * X - inches/s
     * Y - inches/s
     * Rotation2d rad/s
     * @return
     */
    public Pose2d getRobotRelativeVelocityAmerican()
    {
        Translation2d fieldRelativeVelocity =
                new Translation2d(
                        pinpoint.getVelX(DistanceUnit.INCH),
                        pinpoint.getVelY(DistanceUnit.INCH)
                );

        Translation2d robotRelativeVelocity =
                fieldRelativeVelocity.rotateBy(
                        new Rotation2d(
                                pinpoint.getHeading(AngleUnit.RADIANS)
                        ).unaryMinus()
                );

        return new Pose2d(
                robotRelativeVelocity.getX(),
                robotRelativeVelocity.getY(),
                new Rotation2d(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
        );
    }




    /**
     * DONT EVER USE RESETPOSANDIMU
     *
     *
     * I only recalibrate imu
     * Veer 14259 — 8/18/2025 8:16 PM
     * and then setPosition works?
     * Tony | 25631 | 8610a — 8/18/2025 8:16 PM
     * yeah
     * Veer 14259 — 8/18/2025 8:16 PM
     * whaddafuq
     * Tony | 25631 | 8610a — 8/18/2025 8:16 PM
     * if I reset pos and imu then it goes to 0 0 0 no matter what I set it to
     */
//    public void resetPosAndIMU() {
//        if (pinpoint.getDeviceStatus() != GoBildaPinpoint.DeviceStatus.READY)
//        {
//            throw new RuntimeException("Chat pinpoint is not ready");
//        }
//        pinpoint.resetPosAndIMU();
//        lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
//        this.pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
//    }



    /**
     * Warning - will completely break position!!
     */
    public void resetYaw() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpoint.getPosX(DistanceUnit.MM), pinpoint.getPosY(DistanceUnit.MM), AngleUnit.RADIANS, 0));
    }

    public void setYaw(double radians) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpoint.getPosX(DistanceUnit.MM), pinpoint.getPosY(DistanceUnit.MM), AngleUnit.RADIANS, radians));
    }

    /**
     * @param x In inches.
     * @param y In inches.
     */
    public void setPositionTranslation(double x, double y) {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
        {
            throw new RuntimeException("Chat pinpoint is not ready");
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES)));
        lastGoodPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
        this.pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }

    public void setPositionPose(Pose2d pose)
    {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
        {
            throw new RuntimeException("Chat pinpoint is not ready");
        }
        Log.i("pinpoint debug", converter.toSDKPose(pose).toString());
        pinpoint.setPosition(converter.toSDKPose(pose));
        lastGoodPose = converter.toSDKPose(pose);
        this.pose = converter.toSDKPose(pose);
    }

    public boolean isDoneCalibration() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }



    private static class converter
    {
        /**
         * Input Units in Inches and Degrees
         * @param pose FTCLib {@link Pose2d} object
         * @return SDK {@link Pose2D} object
         */
        public static Pose2D toSDKPose(Pose2d pose)
        {
            return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.DEGREES, pose.getRotation().getDegrees());
        }
    }
}
