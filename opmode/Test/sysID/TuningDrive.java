package org.firstinspires.ftc.teamcode.opmode.Test.sysID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
//@Disabled
//@Deprecated
@TeleOp(name = "Tuning Drive", group="3")
//@Config
public class TuningDrive extends Robot {
    public ElapsedTime loopTimer = new ElapsedTime();

    static double largestLinearVelocity = 0;
    static double largestAngularVelocityRads = 0;
    static double largestLinearAccel = 0;
    static double largestAngularAccelRads = 0;

    double lastLinearVelo = 0;
    double lastAngularVelo = 0;
    double measurementCounter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(DriveMode.SIMPLEDRIVE, Alliance.RED);

        waitForStart();
        loopTimer.reset();
        while (!isStopRequested() && opModeIsActive()) {
            mecanumSubsystem.driveRobotCentricWithPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (measurementCounter++ < 3)
            {
                continue;
            }

            //Because for some reason with doubles NaN is a massive number or smth, meaning it messes up your data.
            if (Double.isNaN(pinpointSubsystem.getFieldRelativeVelocityMetric().getX())
                ||
                Double.isNaN(pinpointSubsystem.getFieldRelativeVelocityMetric().getY())
                ||
                Double.isNaN(pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians()))
            {
                continue;
            }

            largestLinearAccel = Math.max(largestLinearAccel, Math.abs(pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getNorm() - lastLinearVelo)/Math.max(loopTimer.seconds(), 1e-3));
            largestAngularAccelRads = Math.max(largestAngularAccelRads, Math.abs(pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians() - lastAngularVelo)/Math.max(loopTimer.seconds(), 1e-3));

            lastLinearVelo = pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getNorm();
            lastAngularVelo = pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians();
            largestLinearVelocity = Math.max(largestLinearVelocity, lastLinearVelo);
            largestAngularVelocityRads = Math.max(largestAngularVelocityRads, Math.abs(lastAngularVelo));

            telemetry.addData("Sticky Largest Linear Velocity m/s", largestLinearVelocity);
            telemetry.addData("Sticky Largest Angular Velocity rad/s", largestAngularVelocityRads);
            telemetry.addData("Sticky Largest Linear Acceleration m/s^2", largestLinearAccel);
            telemetry.addData("Sticky Largest Angular Acceleration rad/s^2", largestAngularAccelRads);

            telemetry.addData("Loop time", loopTimer.milliseconds());
            loopTimer.reset();


            if (gamepad1.aWasPressed())
            {
                largestLinearVelocity = 0;
                largestAngularVelocityRads = 0;
                largestLinearAccel = 0;
                largestAngularAccelRads = 0;
            }
            update();
        }
    }
}
