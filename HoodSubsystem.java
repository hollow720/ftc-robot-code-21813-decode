package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.DoubleSupplier;

public class HoodSubsystem extends TelemetrySubsystem {
    ServoImplEx hood;

    public HoodSubsystem(HardwareMap hardwareMap, DoubleSupplier hoodPositionSupplier)
    {
        hood = hardwareMap.get(ServoImplEx.class, "hood");

        this.setDefaultCommand(
                this.setHoodCContinuous(hoodPositionSupplier)
        );
    }

    private void setHoodPosition(Double position) {
        position = MathUtil.clamp(position, ShooterConstants.retractedHoodPosition, ShooterConstants.fullExtensionHoodPosition);
        hood.setPosition(position);
    }


    public InstantCommand setHoodC(DoubleSupplier pos)
    {
        return new InstantCommand(
                () -> {
                    this.setHoodPosition(pos.getAsDouble());
                }
        );
    }

    public RunCommand setHoodCContinuous(DoubleSupplier pos)
    {
        return new RunCommand(
                () -> {
                    this.setHoodPosition(pos.getAsDouble());
                },
                this
        );
    }
}
