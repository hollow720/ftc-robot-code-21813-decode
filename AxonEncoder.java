package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonEncoder {
    final AnalogInput analogInput;
    private double offsetDegrees = 0.0;

    public AxonEncoder(HardwareMap hardwareMap, String encoderName)
    {
        this.analogInput = hardwareMap.get(AnalogInput.class, encoderName);
    }

    public void setOffset(double degrees)
    {
        this.offsetDegrees = degrees;
    }

    private double getRevolutions()
    {
        return this.analogInput.getVoltage() / 3.3;
    }

    public double getPositionRadians()
    {
        return getRevolutions() * 2.0 * Math.PI;
    }

    public double getPositionDegrees()
    {
        return this.getPositionRadians() * 360.0/(2.0 * Math.PI);
    }

    @Deprecated
    public double getVoltageDEBUGONLY()
    {
        return this.analogInput.getVoltage();
    }
}
