package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;

@Config
public class IntakeSubsystem extends TelemetrySubsystem {
    DcMotorImplEx intake;
    ElapsedTime currentTimer;
    LowPassFilter currentFiltered = new LowPassFilter();

    public static double threeBallStallThreshold = 4.6;
    public static double twoBallThreshold = 3.2;
    public static double zeroBallThreshold = 2.2;

    ColorSensor color1;
    ColorSensor color2;

    private boolean running = false;

    public Trigger threeBalls = new Trigger(() -> {
        return this.getCachedAmps() > threeBallStallThreshold;
    });

    public Trigger twoBalls = new Trigger(() -> {
        return this.getCachedAmps() > twoBallThreshold && this.getCachedAmps() < threeBallStallThreshold;
    });

    public Trigger probablyEmpty = new Trigger(() -> {
        return this.getCachedAmps() < zeroBallThreshold;
    });

    public IntakeSubsystem(HardwareMap hardwareMap, boolean reversed)
    {
        intake = hardwareMap.get(DcMotorImplEx.class, "intake");
        intake.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        color1 = hardwareMap.get(ColorSensor.class, "color1");
//        color2 = hardwareMap.get(ColorSensor.class, "color2");

        currentTimer = new ElapsedTime();
    }

    @Override
    public void periodic() {
        if (running)
        {
            currentTimer.reset();
            currentFiltered.calculate(this.getCurrentAmps());
        }

        telemetryC.accept("Low Pass Intake Amperage", this.getCachedAmps());
        telemetryC.accept("3 balls", threeBalls.get());
        telemetryC.accept("2 balls", twoBalls.get());
        telemetryC.accept("empty balls", probablyEmpty.get());

//        telemetryC.accept("Color 1 Luminance", color1.alpha());
//        telemetryC.accept("Color 2 Luminance", color2.alpha());
//        telemetryC.accept("Color 1 r", color1.red());
//        telemetryC.accept("Color 1 g", color1.green());
//        telemetryC.accept("Color 1 b", color1.blue());
    }

    public DcMotor getIntakeMotor()
    {
        return this.intake;
    }

    public double getCachedAmps()
    {
        return this.currentFiltered.sample();
    }

    private double getCurrentAmps()
    {
        return this.intake.getCurrent(CurrentUnit.AMPS);
    }

    private void setPower(double power)
    {
        if (Math.abs(power) > 0.1)
        {
            running = true;
        }
        if (Math.abs(power) < 0.1)
        {
            running = false;
        }

        this.intake.setPower(power);
    }

    /**
     * Clips from -1.0 to 1
     * @param power
     * @return
     */
    public InstantCommand openLoop(double power)
    {
        return new InstantCommand(
                () ->
                {
                    this.setPower(power);
                }
        );
    }

    public InstantCommand intake()
    {
        return this.openLoop(1.0);
    }

    public InstantCommand stop()
    {
        return this.openLoop(0.0);
    }

    public InstantCommand hold()
    {
        return this.openLoop(.05);
    }

    public InstantCommand eject()
    {
        return this.openLoop(-1.0);
    }
}
