package org.firstinspires.ftc.teamcode.opmode.Test.testdrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.util.Util;

@Disabled
@TeleOp(group="1",name="VelocityDrive")
@Config
public class VelocityDrive extends Robot {
    public ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeInInit())
        {
            super.initDebug();
        }

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // Read from volatile memory
            double y = -gamepad1.left_stick_y; //Reversed because joystick's on controller's have Y inverted
            double x = -gamepad1.left_stick_x; //Reversed because XYZ mapping to NWU reverses
            double rx = -gamepad1.right_stick_x; //Reversed because rotation in Configuration Space is CCW

            double cy = y;
            double cx = x;
            double crx = Util.quadraticJoystickCurve(rx);

            mecanumSubsystem.driveRobotCentricWithVelocity(
                    new ChassisSpeed(
                            cy * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble(),
                            cx * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble(),
                            crx * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble()));

            telemetry.addData("Pinpoint Position", pinpointSubsystem.getPose());
            telemetry.addData("Target X velocity", y);
            telemetry.addData("Target Y velocity", x);
            telemetry.addData("Target Angular velocity (CCW)", crx * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble());
            telemetry.addData("Actual X velocity", pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getX());
            telemetry.addData("Actual Y velocity", pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getY());
            telemetry.addData("Actual Angular velocity", pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians());
            telemetry.addData("Loop time", loopTimer.milliseconds());

            if (gamepad1.xWasPressed())
            {
                stickyDebug = true;
            }

            loopTimer.reset();
            update();
        }

        end();
    }
}