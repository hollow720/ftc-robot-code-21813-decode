package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

public class SpoilerSubsystem extends TelemetrySubsystem {
    Servo spoiler;

    public SpoilerSubsystem(HardwareMap hardwareMap)
    {
        this.spoiler = hardwareMap.get(Servo.class, "spoiler");

        this.spoiler.setPosition(0.5);
    }

    public void pointUp()
    {
        this.spoiler.setPosition(0.3);
    }

    public void pointFlat()
    {
        this.spoiler.setPosition(0.5);
    }
}
