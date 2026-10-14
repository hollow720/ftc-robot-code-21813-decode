package org.firstinspires.ftc.teamcode.opmode.Test.sysID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

@Disabled
@Deprecated
@Autonomous(name = "Max Velo Test", group="3")
@Config
public class MaxVeloTest extends Robot {

    public static long timeToFullms = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // Angular Max Velo
            mecanumSubsystem.driveRobotCentricWithPower(0, 0, 1);
            Thread.sleep(4000);
            telemetry.addData("Max Angular Velocity CCW (rad/s): ", pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians());
            telemetry.update();
            mecanumSubsystem.driveRobotCentricWithPower(0, 0, 0);
            Thread.sleep(1000);

            // Linear Max Velo
            mecanumSubsystem.driveRobotCentricWithPower(1, 0, 0);
            Thread.sleep(timeToFullms);
            telemetry.addData("Max Angular Velocity CCW (rad/s): ", pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians());
            telemetry.update();
            mecanumSubsystem.driveRobotCentricWithPower(0, 0, 0);
            Thread.sleep(1000);
            mecanumSubsystem.driveRobotCentricWithPower(-1, 0, 0);
            Thread.sleep(timeToFullms);
            mecanumSubsystem.driveRobotCentricWithPower(0, 0, 0);
            Thread.sleep(1000);
        }
    }
}
