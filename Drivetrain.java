package org.firstinspires.ftc.teamcode.subsystems.design;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;

public interface Drivetrain extends Subsystem {
    public void driveRobotCentricWithPower(double x, double y, double rx);

    public void driveRobotCentricWithVelocity(ChassisSpeed cs);

    public void driveFieldCentricWithVelocity(double vx, double vy, double vw);
}
