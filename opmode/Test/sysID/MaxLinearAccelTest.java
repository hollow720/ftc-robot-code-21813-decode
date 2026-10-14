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
@Autonomous(name = "Max Linear Accel Test", group="3")
@Config
public class MaxLinearAccelTest extends Robot {

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
                        mecanumSubsystem.driveRobotCentricWithPower(1, 0, 0);
                        timer.reset();
                        total.reset();
                    }

                    @Override
                    public void execute() {
                        double newVal = pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getNorm();
                        if (Math.abs(lastAngVelo - newVal) >= 0.04)
                        {
                            timer.reset();
                            lastAngVelo = newVal;
                        }
                        telemetry.addData("Linear Vel", pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getNorm());
                    }

                    @Override
                    public boolean isFinished() {
                        if (timer.milliseconds() > 100)
                        {
                            telemetry.addData("max linear accel m/s^2", DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble()/total.seconds());
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
