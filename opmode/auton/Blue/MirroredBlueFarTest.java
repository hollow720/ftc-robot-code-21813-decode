package org.firstinspires.ftc.teamcode.opmode.auton.Blue;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedFarAutoConstants;

@Autonomous(name="blue 6 partner")
public class MirroredBlueFarTest extends Robot {


    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.BLUE;
        initialize(DriveMode.VELODRIVE, Pathing.mirrorAlliancePose(RedFarAutoConstants.P1S), this.alliance);

        BezierCommand Path1Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory1)
                .invertAcrossAlliance();

        BezierCommand Path2Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory2)
                .invertAcrossAlliance();

        BezierCommand Path3Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3)
                .invertAcrossAlliance();

        BezierCommand Path4Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4)
                .invertAcrossAlliance();

        BezierCommand Path5Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory1);

        BezierCommand Path6Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory2);

        BezierCommand Path7Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3);

        BezierCommand Path8Command = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4);

        while (opModeInInit())
        {
            super.initDebug();
        }

        waitForStart();

        CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            Path1Command,
                            Path2Command,
                            Path3Command,
                            Path4Command
            )
        );

        while (opModeIsActive() && !isStopRequested())
        {
            update();
            telemetry.addData("Pinpoint Pose", pinpointSubsystem.getPose());
        }

        end();
    }
}
