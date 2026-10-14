package org.firstinspires.ftc.teamcode.control.pathing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public abstract class Follower extends LogAccess {

    public void start(Trajectory trajectory, ElapsedTime sharedTimer, PinpointSubsystem pinpointSubsystem)
    {

    }

    public void timeOverload()
    {

    }

    public ChassisSpeed calculate()
    {
        return new ChassisSpeed();
    }

    public boolean isFinished(){
        return false;
    }
}
