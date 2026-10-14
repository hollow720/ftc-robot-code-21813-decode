package org.firstinspires.ftc.teamcode.control.commands;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.AimingConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.util.WPIPID;

public class DirectToPointCommand extends TelemetryCommand {
    WPIPID vpxController;
    WPIPID vpyController;
    WPIPID vpwController;
    MecanumDriveSubsystem mecanumSubsystem;
    PinpointSubsystem pinpointSubsystem;
    Pose2d targetPose;
    public DirectToPointCommand(MecanumDriveSubsystem mecanumSubsystem, PinpointSubsystem pinpointSubsystem, Pose2d targetPose)
    {
        vpxController = new WPIPID(AimingConstants.translationalKp_x, 0, 0);
        vpyController = new WPIPID(AimingConstants.translationalKp_y, 0, 0);
        vpwController = new WPIPID(AimingConstants.rotationalKp, 0.0, 0.0);
        this.vpwController.enableContinuousInput(-Math.PI, Math.PI); //The reason why its [-PI, PI] is because its wrapping ERROR not the absolute heading :)

        this.mecanumSubsystem = mecanumSubsystem;
        this.pinpointSubsystem = pinpointSubsystem;
        this.targetPose = targetPose;
        setTargetPose(targetPose);

        addRequirements(mecanumSubsystem);
    }

    @Override
    public void initialize() {

    }

    public void setTargetPose(Pose2d targetPose)
    {
        this.targetPose = targetPose;
        vpxController.setSetpoint(targetPose.getX());
        vpyController.setSetpoint(targetPose.getY());
        vpwController.setSetpoint(targetPose.getHeading());
    }

    public DirectToPointCommand setXTolerance(DistanceUnit unit, double positionTolerance)
    {
        vpxController.setTolerance(unit.toInches(positionTolerance));

        return this;
    }

    public DirectToPointCommand setYTolerance(DistanceUnit unit, double positionTolerance)
    {
        vpyController.setTolerance(unit.toInches(positionTolerance));

        return this;
    }

    public DirectToPointCommand setRotationTolerance(AngleUnit unit, double rotationTolerance)
    {
        vpwController.setTolerance(unit.toRadians(rotationTolerance));

        return this;
    }

    @Override
    public void execute() {
        if (pinpointSubsystem.getPose().minus(this.targetPose).getTranslation().getNorm() < 30.0)
        {
            this.vpxController.setP(AimingConstants.closeTranslationalKp_x);
            this.vpyController.setP(AimingConstants.closeTranslationalKp_y);
        }

        double xcontrol = vpxController.calculate(pinpointSubsystem.getPose().getX());
        double ycontrol = vpyController.calculate(pinpointSubsystem.getPose().getY());
        double wcontrol = vpwController.calculate(pinpointSubsystem.getPose().getHeading());


        if (vpxController.atSetpoint())
        {
            xcontrol = 0.0;
        }
        if (vpyController.atSetpoint())
        {
            ycontrol = 0.0;
        }
        if (vpwController.atSetpoint())
        {
            wcontrol = 0.0;
        }
        wcontrol = wcontrol + Math.signum(wcontrol) * AimingConstants.feedforwardInertiaRot;

        telemetryC.accept("X control effort: ", xcontrol);
        telemetryC.accept("Y control effort: ", ycontrol);
        telemetryC.accept("w control effort: ", wcontrol);


        mecanumSubsystem.driveFieldCentricWithVelocity(
                xcontrol, ycontrol, wcontrol);
    }

    @Override
    public void end(boolean interrupted) {
        mecanumSubsystem.driveFieldCentricWithVelocity(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return vpxController.atSetpoint() && vpyController.atSetpoint() && vpwController.atSetpoint();
    }
}


