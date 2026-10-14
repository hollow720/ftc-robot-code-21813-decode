package org.firstinspires.ftc.teamcode.opmode.Test.sysID;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

@Disabled
@Deprecated
@Autonomous(name = "Max Angular Accel Test", group="3")
@Config
public class MaxAngularAccelTest extends Robot {

    public static long timeToFullms = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        CommandScheduler.getInstance().schedule(
                new CommandBase() {
                    ElapsedTime timer = new ElapsedTime();
                    ElapsedTime total = new ElapsedTime();
                    public double lastAngVelo = 0;

                    @Override
                    public void initialize() {
                        mecanumSubsystem.driveRobotCentricWithPower(0, 0, 1);
                        timer.reset();
                        total.reset();
                    }

                    @Override
                    public void execute() {
                        double newVal = pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians();
                        if (Math.abs(lastAngVelo - newVal) >= 0.02)
                        {
                            timer.reset();
                            lastAngVelo = newVal;

                        }

                        telemetry.addData("Ang Vel", pinpointSubsystem.getFieldRelativeVelocityMetric().getRotation().getRadians());
                    }

                    @Override
                    public boolean isFinished() {
                        if (timer.milliseconds() > 200)
                        {
                            telemetry.addData("max angular accel rad/s^2", DriveConstants.maxWheelAngularVelocityRadiansPerSecond.getAsDouble()/total.seconds());
                            mecanumSubsystem.driveRobotCentricWithPower(0, 0, 0);
                            return true;
                        }

                        return false;
                    }
                }
        );

        while (!isStopRequested() && opModeIsActive()) {
            update();
        }
    }
}
