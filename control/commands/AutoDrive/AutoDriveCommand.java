package org.firstinspires.ftc.teamcode.control.commands.AutoDrive;

import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants.*;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.TelemetryCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class AutoDriveCommand extends TelemetryCommand {
    final Robot.Alliance alliance;
    final PinpointSubsystem pinpointSubsystem;
    final MecanumDriveSubsystem mecanumDriveSubsystem;

    BezierCommand bezierCommand = null;
    ElapsedTime timer = new ElapsedTime();

    public AutoDriveCommand(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem)
    {
        this.alliance = alliance;
        this.pinpointSubsystem = pinpointSubsystem;
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
    }

    @Override
    public void initialize() {
        Bezier dynamicPath = new Bezier(
            this.pinpointSubsystem.getPose(),
            gateEnd
        );


        Trajectory dynamicTrajectory = new Trajectory(
            dynamicPath,
            highSpeedMotionProfile,
            gateEndingConfig,
            this.pinpointSubsystem.getPose(),
            gateEnd
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

    @Override
    public void execute() {
        this.telemetryC.accept(">>> AUTO [UNOVERLOADED] PATHING ACTIVE <<<", timer.seconds());
    }

    @Override
    public void end(boolean interrupted) {
        bezierCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return bezierCommand.isFinished();
    }
}
