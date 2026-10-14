/* Copyright (c) 2024 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.CameraConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This OpMode demonstrates the basics of using multiple vision portals simultaneously
 */
@TeleOp(name = "AprilTagMultiPortalTest", group = "1")
public class AprilTagMultiPortalTest extends LinearOpMode
{
    VisionPortal portal1;
    VisionPortal portal2;

    AprilTagProcessor aprilTagProcessor1;
    AprilTagProcessor aprilTagProcessor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Because we want to show two camera feeds simultaneously, we need to inform
        // the SDK that we want it to split the camera monitor area into two smaller
        // areas for us. It will then give us View IDs which we can pass to the individual
        // vision portals to allow them to properly hook into the UI in tandem.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // We extract the two view IDs from the array to make our lives a little easier later.
        // NB: the array is 2 long because we asked for 2 portals up above.
        int portal1ViewId = viewIds[0];
        int portal2ViewId = viewIds[1];

        // If we want to run AprilTag detection on two portals simultaneously,
        // we need to create two distinct instances of the AprilTag processor,
        // one for each portal. If you want to see more detail about different
        // options that you have when creating these processors, go check out
        // the ConceptAprilTag OpMode.

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
        CameraConstants.CameraConfig rc = CameraConstants.rightCamera;
        aprilTagProcessor1 =
                new AprilTagProcessor
                        .Builder()
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                        .setDrawAxes(true)
                        .setDrawCubeProjection(true)
                        .setDrawTagOutline(true)
                        .setLensIntrinsics(rc.Fx(), rc.Fy(), rc.Cx(), rc.Cy())
                        .setCameraPose(
                                rc.getTranslation(),
                                rc.getOrientation()
                        )
                        .setNumThreads(3) //default
                        .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor1.setDecimation(rc.getDecimation());
        aprilTagProcessor1.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN);

//        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();

        // Now we build both portals. The CRITICAL thing to notice here is the call to
        // setLiveViewContainerId(), where we pass in the IDs we received earlier from
        // makeMultiPortalView().

        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "rightCamera"))
                .setLiveViewContainerId(portal1ViewId)
                .setCameraResolution(rc.getResolution()) //px
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor1)
                .build();
//        portal2 = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setLiveViewContainerId(portal2ViewId)
//                .addProcessor(aprilTagProcessor2)
//                .build();

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime loopTimer = new ElapsedTime();

        // Main Loop
        while (opModeIsActive())
        {
            loopTimer.reset();
            // Just show some basic telemetry to demonstrate both processors are working in parallel
            // on their respective cameras. If you want to see more detail about the information you
            // can get back from the processor, you should look at ConceptAprilTag.
            if (gamepad1.left_bumper)
            {
                portal1.stopStreaming();
            }
            if (gamepad1.right_bumper)
            {
//                portal2.stopStreaming();
            }

            if (gamepad1.a)
            {
                portal1.resumeStreaming();
//                portal2.resumeStreaming();
            }

            telemetry.addData("Vision Portal 1 State", portal1.getCameraState());
//            telemetry.addData("Vision Portal 2 State", portal2.getCameraState());

            telemetry.addData("Loop Timer (ms)", loopTimer.milliseconds());

            telemetry.addData("Number of tags in Camera 1", aprilTagProcessor1.getDetections().size());
//            telemetry.addData("Number of tags in Camera 2", aprilTagProcessor2.getDetections().size());

            telemetry.addData("Camera 1 FPS", portal1.getFps());
//            telemetry.addData("Camera 2 FPS", portal2.getFps());

            telemetry.addData("Apriltag Processor Camera 1 Avg Solve Time (ms)", aprilTagProcessor1.getPerTagAvgPoseSolveTime());
//            telemetry.addData("Apriltag Processor Camera 2 Avg Solve Time (ms)", aprilTagProcessor2.getPerTagAvgPoseSolveTime());



            List<AprilTagDetection> currentDetections = aprilTagProcessor1.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(),
                            detection.robotPose.getOrientation().getRoll(),
                            detection.robotPose.getOrientation().getYaw()));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                telemetry.addData("Decision Margin", detection.decisionMargin);
                telemetry.addData("Hamming", detection.hamming);

            }

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");


//            aprilTagProcessor2.getFreshDetections().stream().forEach(tag -> {
//                telemetry.addLine("------------");
//                telemetry.addData("Camera 1 Robot Pose", tag.robotPose);
//                telemetry.addData("Camera 1 Ftc Pose", tag.ftcPose);
//                telemetry.addData("Camera 1 Raw Pose", tag.rawPose);
//                telemetry.addData("Camera 1 Decision Margin", tag.decisionMargin);
//                telemetry.addLine("------------");
//            });
            telemetry.update();
        }
    }
}
