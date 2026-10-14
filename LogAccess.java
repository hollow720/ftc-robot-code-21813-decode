package org.firstinspires.ftc.teamcode.control.pathing;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public abstract class LogAccess {
    protected Supplier<BiConsumer<String, Object>> telemetrySupplier = Robot.getTelemetry;
    protected BiConsumer<String, Object> telemetryC;
    protected Supplier<Runnable> updateSupplier = Robot.updateSupplier;
    protected Runnable updateTelemetry = () -> {};

    protected LogAccess()
    {
        telemetryC = telemetrySupplier.get();
        updateTelemetry = updateSupplier.get();
    }

    public void refreshTelemetryAccess() { telemetryC = telemetrySupplier.get(); }
}
