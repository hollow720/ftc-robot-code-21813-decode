package org.firstinspires.ftc.teamcode.opmode.Test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.PathingConstants;
import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

//@Config
@Autonomous(group="2", name="24in bezier test")
public class BezierTest extends Robot {
    private static Pose2d P1 = new Pose2d(0, 0, new Rotation2d(0));
    private static Pose2d P2 = new Pose2d(30, 0, new Rotation2d(0));

    Bezier testBezier = new Bezier(P1, P2);

    Trajectory.mpParams testParams =
//            new Trajectory.mpParams(
//                    AimingConstants.maxVx,
//                    AimingConstants.maxAxy,
//                    AimingConstants.maxVw,
//                    AimingConstants.maxAw);
            new Trajectory.mpParams(
                    4,
                    2.5
            );
    Trajectory.EndingConfig testEndingConfig = new Trajectory.EndingConfig(PathingConstants.minimumParametricEnding, PathingConstants.defaultLinger);

    Trajectory testTrajectory = new Trajectory(testBezier, testParams, testEndingConfig, P1.getRotation(), P2.getRotation());



    @Override
    public void runOpMode() throws InterruptedException {
        initialize(P1);
        testTrajectory.refreshTelemetryAccess();

        BezierCommand testBezierCommand = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, testTrajectory);

        SequentialCommandGroup group = new SequentialCommandGroup(
                testBezierCommand
        );

        while (opModeInInit())
        {
            this.initDebug();
        }

        waitForStart();
        CommandScheduler.getInstance().schedule(group);
        while (!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("Pinpoint Position", pinpointSubsystem.getPose());
            update();
        }

        end();
    }
}
