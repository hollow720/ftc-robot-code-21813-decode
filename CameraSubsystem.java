package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import android.util.Pair;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.CameraConstants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class CameraSubsystem extends TelemetrySubsystem {
    final AprilTagProcessor aprilTagProcessor;
    final VisionPortal visionPortal;

    CameraConstants.CameraConfig config;
    ElapsedTime timer;

    Optional<AprilTagDetection> cachedDetection = Optional.empty();
    double cacheTime = 0;

    Optional<Integer> cachedObelisk = Optional.empty();

    public CameraSubsystem(HardwareMap hardwareMap, CameraConstants.CameraConfig config, PinpointSubsystem pinpointSubsystem) {
        this.config = config;

        aprilTagProcessor =
                new AprilTagProcessor
                        .Builder()
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                        .setDrawAxes(true)
                        .setDrawCubeProjection(true)
                        .setDrawTagOutline(true)
                        .setLensIntrinsics(config.Fx(), config.Fy(), config.Cx(), config.Cy())
                        .setCameraPose(
                                config.getTranslation(),
                                config.getOrientation()
                        )
                        .setNumThreads(3) //This is a quad core system so maybe 2 threads per camera?
                        .build();

        aprilTagProcessor.setDecimation(config.getDecimation());
        aprilTagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "rightCamera"))
//                .setLiveViewContainerId(portal1ViewId)
                .setCameraResolution(config.getResolution()) //px
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();

        setExposureWithTimeout(visionPortal, config.getExposureMs());
        setGainWithTimeout(visionPortal, config.getGain());

        timer = new ElapsedTime();

//        this.setDefaultCommand(
//                new defaultLocalizeCommand(pinpointSubsystem)
//        );
    }

    public void setExposureWithTimeout(VisionPortal visionPortal, double exposureMs)
    {

        ElapsedTime timer = new ElapsedTime();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && timer.milliseconds() < 300)
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long)exposureMs, TimeUnit.MILLISECONDS);
        }
    }

    public void setGainWithTimeout(VisionPortal visionPortal, int gain)
    {

        ElapsedTime timer = new ElapsedTime();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && timer.milliseconds() < 300)
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            {
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

                gainControl.setGain(gain);
            }
    }

    @Override
    public void periodic() {
        Pair<Optional<AprilTagDetection>, Double> pollResult = this.pollDetector();
        if (pollResult.first.isPresent()) {
            this.cachedDetection = pollResult.first;
            this.cacheTime = pollResult.second;
        }
        if (timer.milliseconds() - cacheTime > CameraConstants.staleSampleThreshold)
            cachedDetection = Optional.empty();

//        if (cachedDetection.isPresent()) {
//            int i = 1;
//            for (AprilTagDetection a : cachedDetection.get()) {
//                telemetryC.accept(String.format("Cached Detection %d", i++), a.robotPose);
//            }
//        }
    }

    public Pair<Optional<AprilTagDetection>, Double> pollDetector() {
        Optional<List<AprilTagDetection>> currentDetections = Optional.ofNullable(aprilTagProcessor.getDetections());

        if (currentDetections.isPresent()) {
            for (AprilTagDetection detection : currentDetections.get()) {
                //Debug
//                if (detection.metadata == null || detection.metadata.name == null) continue;
//                telemetryC.accept(detection.metadata.name, detection.metadata.id);
//                telemetryC.accept("Tag Size", detection.metadata.tagsize);
//                telemetryC.accept("Decision Margin", detection.decisionMargin);
//                telemetryC.accept("Hamming", detection.hamming);
                //

                try {
                    if (detection.metadata.name.contains("Obelisk") ) { // You're looking at Obelisk tag
                        cachedObelisk = Optional.of(detection.metadata.id);
                    }
                }
                catch (Exception e)
                {
                    //i dont care actually
                }

            telemetryC.accept("decision margin log", detection.decisionMargin);
                return new Pair<>(
                        Optional.of(detection),
                        timer.milliseconds()
                );
            }
        }

        return new Pair<>(Optional.empty(), timer.milliseconds());
    }

    /**
     * Returns Cached Pose2d value
     * May return Optional.empty() if there are stale values
     *
     * @return
     */
    public Optional<Pose2d> getPose2d() {
        if (cachedDetection.isPresent()) {
            Pose3D robotPose = cachedDetection.get().robotPose;
            return Optional.of(new Pose2d(new Translation2d(robotPose.getPosition().x, robotPose.getPosition().y), this.correctApriltagRotationWithRotation2d(new Rotation2d(robotPose.getOrientation().getYaw(AngleUnit.RADIANS)))));
        } else {
            return Optional.empty();
        }
    }

    public int numberOfDetectedTags() {
        return this.aprilTagProcessor.getDetections().size();
    }

    /**
     * Comprehensive guide to SDK coordinate frames
     *
     * For the Position class:
     * robot coordinate system
     * X left
     * Y forward
     * Z up
     *
     * For the Orientation class:
     * -> Same as Position
     * CCW
     *
     * For pose2d that we use
     * NWU
     * X forward
     * Y left
     * Z up
     *
     * CCW
     *
     * For the Apriltag coordinate system
     * X left
     * Y Behind
     * Z Up
     *
     * CCW
     *
     * @return
     */
    public Rotation2d correctApriltagRotationWithRotation2d(Rotation2d apriltagOrientation)
    {
        return apriltagOrientation.rotateBy(Rotation2d.fromDegrees(90));
    }

    public initLocalizeCommand getLocalizeCommand(PinpointSubsystem pinpoint)
    {
        return new initLocalizeCommand(pinpoint);
    }

    public class defaultLocalize extends CommandBase
    {
        Command command;

        public defaultLocalize(Command command)
        {
            this.command = command;
        }

        @Override
        public void execute() {
            if (!command.isScheduled())
            {
                CommandScheduler.getInstance().schedule(command);
            }
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class initLocalizeCommand extends CommandBase
    {
        List<Pose3D> storedDetections;
        final PinpointSubsystem storedPinpoint;

        ElapsedTime commandTimer = new ElapsedTime();

        public initLocalizeCommand(PinpointSubsystem pinpointReference)
        {
            this.storedPinpoint = pinpointReference;
            this.addRequirements(CameraSubsystem.this);
        }

        @Override
        public void initialize() {
            commandTimer.reset();
            storedDetections = new LinkedList<>();
        }

        @Override
        public void execute() {
            Pair<Optional<AprilTagDetection>, Double> pollResult = CameraSubsystem.this.pollDetector();
            pollResult.first.ifPresent(
                    aprilTagDetections -> this.storedDetections.add(aprilTagDetections.robotPose));

            telemetryC.accept("Total Stored Detections", storedDetections.size());
        }

        @Override
        public void end(boolean interrupted) {
            double averageX = 0.0;
            double averageY = 0.0;
            double n = storedDetections.size();

            double unitVectorX = 0.0;
            double unitVectorY = 0.0;

            for (Pose3D detection : storedDetections)
            {
                averageX += detection.getPosition().x;
                averageY += detection.getPosition().y;

                Rotation2d correctedApriltagRotation = CameraSubsystem.this.correctApriltagRotationWithRotation2d(
                        new Rotation2d(detection.getOrientation().getYaw(AngleUnit.RADIANS))
                );
                unitVectorX += Math.cos(correctedApriltagRotation.getRadians());
                unitVectorY += Math.sin(correctedApriltagRotation.getRadians());
            }

            averageX /= n;
            averageY /= n;

            Translation2d averagedTranslation
                    = new Translation2d(averageX, averageY);
            Rotation2d averagedRotation
                    = new Rotation2d(unitVectorX, unitVectorY);
//            Rotation2d averagedRotation
//                    = Rotation2d.fromDegrees(storedDetections.get(storedDetections.size()-1).getOrientation().getYaw(AngleUnit.DEGREES));
            Pose2d averagedPose
                    = new Pose2d(averagedTranslation, averagedRotation);

            if (n!=0)
            {
                storedPinpoint.setPositionPose(averagedPose);
            }
        }

        @Override
        public boolean isFinished() {
            return this.commandTimer.milliseconds() >= 1000;
        }
    }

    public class defaultLocalizeCommand extends CommandBase
    {
        List<Pose3D> storedDetections;
        final PinpointSubsystem storedPinpoint;

        ElapsedTime commandTimer = new ElapsedTime();

        public defaultLocalizeCommand(PinpointSubsystem pinpointReference)
        {
            this.storedPinpoint = pinpointReference;
            this.addRequirements(CameraSubsystem.this);
        }

        @Override
        public void initialize() {
            commandTimer.reset();
            storedDetections = new LinkedList<>();

            Pair<Optional<AprilTagDetection>, Double> pollResult = CameraSubsystem.this.pollDetector();
            pollResult.first.ifPresent(
                    aprilTagDetections -> this.storedDetections.add(aprilTagDetections.robotPose));
        }

        @Override
        public void execute() {

        }

        @Override
        public void end(boolean interrupted) {
            double averageX = 0.0;
            double averageY = 0.0;
            double n = storedDetections.size();

            double unitVectorX = 0.0;
            double unitVectorY = 0.0;

            for (Pose3D detection : storedDetections)
            {
                averageX += detection.getPosition().x;
                averageY += detection.getPosition().y;

                Rotation2d correctedApriltagRotation = CameraSubsystem.this.correctApriltagRotationWithRotation2d(
                        new Rotation2d(detection.getOrientation().getYaw(AngleUnit.RADIANS))
                );
                unitVectorX += Math.cos(correctedApriltagRotation.getRadians());
                unitVectorY += Math.sin(correctedApriltagRotation.getRadians());
            }

            averageX /= n;
            averageY /= n;

            Translation2d averagedTranslation
                    = new Translation2d(averageX, averageY);
            Rotation2d averagedRotation
                    = new Rotation2d(unitVectorX, unitVectorY);
//            Rotation2d averagedRotation
//                    = Rotation2d.fromDegrees(storedDetections.get(storedDetections.size()-1).getOrientation().getYaw(AngleUnit.DEGREES));
            Pose2d averagedPose
                    = new Pose2d(averagedTranslation, averagedRotation);

            if (n!=0)
            {
                storedPinpoint.setPositionPose(averagedPose);
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}

/**
 * Variables to store the position and orientation of the camera on the robot. Setting these
 * values requires a definition of the axes of the camera and robot:
 *
 * Camera axes:
 * Origin location: Center of the lens
 * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
 *
 * Robot axes (this is typical, but you can define this however you want):
 * Origin location: Center of the robot at field height
 * Axes orientation: +x right, +y forward, +z upward
 *
 * Position:
 * If all values are zero (no translation), that implies the camera is at the center of the
 * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
 * inches above the ground - you would need to set the position to (-5, 7, 12).
 *
 * Orientation:
 * If all values are zero (no rotation), that implies the camera is pointing straight up. In
 * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
 * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
 * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
 * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
 */
