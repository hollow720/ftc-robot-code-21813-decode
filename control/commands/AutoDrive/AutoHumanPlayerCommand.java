package org.firstinspires.ftc.teamcode.control.commands.AutoDrive;

import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.gateEndingConfig;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.highSpeedMotionProfile;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.hp1Control;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.hp2Control;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.hpEndingConfig;
import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.hpRedEnd;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class AutoHumanPlayerCommand extends AutoDriveCommand {
    public boolean reversed = false;

    public AutoHumanPlayerCommand(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem, boolean reversed)
    {
        super(alliance, pinpointSubsystem, mecanumDriveSubsystem);

        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        Bezier dynamicPath = new Bezier(
            this.pinpointSubsystem.getPose(),
            hp1Control,
            hp2Control,
            hpRedEnd
        );

        Trajectory dynamicTrajectory = new Trajectory(
            dynamicPath,
            highSpeedMotionProfile,
            hpEndingConfig,
            this.pinpointSubsystem.getPose(),
            hpRedEnd
        );

        this.bezierCommand = new BezierCommand(
                pinpointSubsystem, mecanumDriveSubsystem, dynamicTrajectory
        );

        if (alliance.equals(Robot.Alliance.BLUE))
        {
            this.bezierCommand = this.bezierCommand.invertAcrossAlliance();
        }

        if (reversed)
        {
            this.bezierCommand = this.bezierCommand.invertAcrossAlliance();
        }

        CommandScheduler.getInstance().schedule(
                this.bezierCommand
        );

        timer.reset();
    }
}
