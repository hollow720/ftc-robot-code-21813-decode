package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

public class TiltSubsystem extends TelemetrySubsystem {
    private final Servo rightTilt;
    private final Servo leftTilt;

    public static double retractedPos = 0.0;
    public static double extendedPos = 1.0;

    public TiltSubsystem(HardwareMap hardwareMap)
    {
        this.rightTilt = hardwareMap.get(Servo.class, "rightTilt");
        this.leftTilt = hardwareMap.get(Servo.class, "leftTilt");

        this.setExtension(retractedPos);
    }

    public void setExtension(double pos)
    {
        this.rightTilt.setPosition(pos);
        this.leftTilt.setPosition(pos);
    }

    public InstantCommand park()
    {
        return new InstantCommand(
                () -> {
                    this.setExtension(extendedPos);
                }
        );
    }

    public InstantCommand retract()
    {
        return new InstantCommand(
                () -> {
                    this.setExtension(retractedPos);
                }
        );
    }
}
