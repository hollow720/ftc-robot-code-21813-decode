package org.firstinspires.ftc.teamcode.control.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.control.pathing.Follower;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;
import org.firstinspires.ftc.teamcode.control.pathing.mpFollower;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class BezierCommand extends TelemetryCommand {
    Trajectory trajectory;
    ElapsedTime sharedTimer = new ElapsedTime();
    Follower follower;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    PinpointSubsystem pinpointSubsystem;

    public BezierCommand(PinpointSubsystem pinpointSubsystem, MecanumDriveSubsystem mecanumDriveSubsystem, Trajectory trajectory)
    {
        this.pinpointSubsystem = pinpointSubsystem;
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.trajectory = trajectory;

        addRequirements(mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {
        this.follower = new mpFollower(); //because its in initialize now command instances are reusable?
        sharedTimer.reset();
        follower.start(trajectory, sharedTimer, pinpointSubsystem);
        follower.timeOverload();
    }

    @Override
    public void execute() {
        ChassisSpeed cs = follower.calculate();
        mecanumDriveSubsystem.driveFieldCentricWithVelocity(cs.getVx(), cs.getVy(), cs.getOmega());
    }

    public double getCompletedFraction()
    {
        return this.sharedTimer.seconds()/this.trajectory.getPathDuration();
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        mecanumDriveSubsystem.driveRobotCentricWithVelocity(new ChassisSpeed());
    }

    public BezierCommand invertAcrossAlliance()
    {
        return new BezierCommand(
                pinpointSubsystem,
                mecanumDriveSubsystem,
                this.trajectory.mirrorAllianceSide()
        );
    }
}
