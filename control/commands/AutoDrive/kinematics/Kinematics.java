package org.firstinspires.ftc.teamcode.control.kinematics;

import java.util.List;

public interface Kinematics {
    public List<Double> toWheelSpeeds(ChassisSpeed cs);
    public List<Double> deSaturateWheelSpeeds(List<Double> wheelSpeeds);
    public ChassisSpeed toChassisSpeed(List<Double> wheelSpeeds);
}
