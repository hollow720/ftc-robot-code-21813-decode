package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

import java.util.function.DoubleSupplier;

public class SOTM extends TelemetrySubsystem {
    PinpointSubsystem pinpointSubsystem;
    Telemetry telemetry;
    Robot.Alliance alliance;
    DoubleSupplier rpmSupplier = () -> 0;

    public SOTM(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, Telemetry telemetry, DoubleSupplier rpmSupplier)
    {
        this.pinpointSubsystem = pinpointSubsystem;
        this.alliance = alliance;
        this.telemetry = telemetry;
        this.rpmSupplier = rpmSupplier;
    }

    @Override
    public void periodic() {
    }

    public Double getRPM()
    {
        return ShooterConstants.interpolateFromShooterDist(this.projectedInchesFromGoalSOTM()).fst;
    }

    public Double getHood()
    {
        return
                Math.max(0.0,
                        ShooterConstants.interpolateFromShooterDist(this.projectedInchesFromGoalSOTM()).snd
                        - Math.abs(this.getRPM() - rpmSupplier.getAsDouble())/10000.0
                );
    }

    public Pose2d getGoalPose()
    {
        switch (alliance)
        {
            case RED:
                return TurretConstants.redGoal;
            case BLUE:
                return TurretConstants.blueGoal;
        }

        return TurretConstants.redGoal;
    }

    public double inchesFromGoal()
    {
        Pose2d goalPose = getGoalPose();

        return this.pinpointSubsystem.getPose().minus(goalPose).getTranslation().getNorm();
    }

    public double projectedInchesFromGoalSOTM()
    {
        Pose2d goalPose = getGoalPose();

        Pose2d robotToGoal = this.iterativeProjectedRobotToField();
        return robotToGoal.minus(goalPose).getTranslation().getNorm();
    }

    public Rotation2d projectedRobotTurnToGoalSOTM()
    {
        Pose2d fieldToGoal = this.getGoalPose();
        Pose2d projectedRobotToField = this.iterativeProjectedRobotToField();

        Pose2d robotToGoal = fieldToGoal.relativeTo(projectedRobotToField);
        Rotation2d robotTurnToGoal = new Rotation2d(robotToGoal.getX(), robotToGoal.getY());

        return robotTurnToGoal;
    }

    public Pose2d iterativeProjectedRobotToField()
    {
        double shotTime = this.iterativeShotTime();

        Pose2d projectedRobotToField = this.projectedRobotToField(shotTime+ShooterConstants.hysteresisComp);

        return projectedRobotToField;
    }

    public double iterativeShotTime()
    {
        double shotTime = ShooterConstants.getShotTime(this.inchesFromGoal());
        Pose2d projectedRobotToField = this.projectedRobotToField(shotTime);
        Pose2d goalPose = getGoalPose();

        for (int i = 0; i < 10; i++)
        {
            double distToGoal = projectedRobotToField.minus(goalPose).getTranslation().getNorm();
            shotTime = ShooterConstants.getShotTime(distToGoal);
            projectedRobotToField = this.projectedRobotToField(shotTime);
        }

        double tempDistance = projectedRobotToField.minus(goalPose).getTranslation().getNorm();
        shotTime = ShooterConstants.getShotTime(tempDistance);

        return shotTime;
    }

    public Pose2d projectedRobotToField(double shotTime)
    {
        Pose2d fieldToRobot = pinpointSubsystem.getPose();
        fieldToRobot = fieldToRobot.plus(new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        Translation2d deltaField = pinpointSubsystem.getVelocityAmerican().getTranslation().times(shotTime);
        return new Pose2d(
                fieldToRobot.getTranslation().plus(deltaField),
                fieldToRobot.getRotation()
        );
    }
}
