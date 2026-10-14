package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MelonboticsEncoder {
    AnalogInput analogInput;

    public double encoderOffsetRevolutions = 0.0;

    public MelonboticsEncoder(HardwareMap hardwareMap)
    {
        analogInput = hardwareMap.get(AnalogInput.class, "encoder");
    }

    public MelonboticsEncoder setOffsetDegrees(double degrees)
    {
        this.encoderOffsetRevolutions = degrees/360.0;

        return this;
    }

    public double getRevolutions()
    {
        return analogInput.getVoltage() / 3.2 + this.encoderOffsetRevolutions;
    }

    public double getDegrees()
    {
        return getRevolutions() * 360.0;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }
}
