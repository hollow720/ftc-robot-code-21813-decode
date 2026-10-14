package org.firstinspires.ftc.teamcode.control.commands.AutoDrive;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.control.commands.SpecialGateBezierCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import static org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants.*;

public class AutoGateCommand extends AutoDriveCommand {
    IntakeSubsystem intakeSubsystem;
    public AutoGateCommand(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem, IntakeSubsystem intakeSubsystem)
    {
        super(alliance, pinpointSubsystem, mecanumDriveSubsystem);

        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        Bezier dynamicPath = new Bezier(
            this.pinpointSubsystem.getPose(),
            P2C1,
            P2C2,
            P2E
        );

        Trajectory dynamicTrajectory = new Trajectory(
            dynamicPath,
            gateMotionProfile,
            gateEndingConfig,
            this.pinpointSubsystem.getPose(),
            gateEnd
        );

        this.bezierCommand = new SpecialGateBezierCommand(
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
        if (this.bezierCommand.getCompletedFraction() > 0.7)
        {
            this.intakeSubsystem.intake();
        }
    }
}
