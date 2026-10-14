package org.firstinspires.ftc.teamcode.control.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class TelemetryCommand extends CommandBase {
    protected Supplier<BiConsumer<String, Object>> telemetrySupplier = Robot.getTelemetry;
    protected BiConsumer<String, Object> telemetryC;

    protected TelemetryCommand()
    {
        telemetryC = telemetrySupplier.get();
    }

//    public void refreshTelemetryAccess() { telemetryC = telemetrySupplier.get(); }
}
