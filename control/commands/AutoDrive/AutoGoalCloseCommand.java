package org.firstinspires.ftc.teamcode.control.commands.AutoDrive;

import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.GoalCloseEnd;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.goalEndingConfig;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.highSpeedMotionProfile;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class AutoGoalCloseCommand extends AutoDriveCommand {
    public AutoGoalCloseCommand(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem)
    {
        super(alliance, pinpointSubsystem, mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {
        Bezier dynamicPath = new Bezier(
            this.pinpointSubsystem.getPose(),
                GoalCloseEnd
        );

        Trajectory dynamicTrajectory = new Trajectory(
            dynamicPath,
            highSpeedMotionProfile,
                goalEndingConfig,
            this.pinpointSubsystem.getPose(),
                GoalCloseEnd
        );

        this.bezierCommand = new BezierCommand(
                pinpointSubsystem, mecanumDriveSubsystem, dynamicTrajectory
        );

        if (alliance.equals(Robot.Alliance.BLUE))
        {
            this.bezierCommand = this.bezierCommand.invertAcrossAlliance();
        }

        CommandScheduler.getInstance().schedule(
                this.bezierCommand
        );

        timer.reset();
    }
}
