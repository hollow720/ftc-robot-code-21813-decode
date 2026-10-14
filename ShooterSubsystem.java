package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends TelemetrySubsystem {
    DcMotorImplEx shooter;
    public static double encoderResolution = 28.0;
    CachingVoltageSensor cache;

    public ShooterSubsystem(HardwareMap hardwareMap, CachingVoltageSensor cache)
    {
        shooter = hardwareMap.get(DcMotorImplEx.class, "shooter");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.cache = cache;
    }

    @Override
    public void periodic() {
        telemetryC.accept("rpm", this.getMotorVelocity());
//        telemetryC.accept("shoot power", this.shooter.getPower());
    }

    private void setPower(double power)
    {
        shooter.setPower(power);
    }

    public InstantCommand setPowerC(double power)
    {
        return new InstantCommand(
                () -> {
                    this.setPower(power);
                },
                this
        );
    }

    public InstantCommand setPowerC(DoubleSupplier power)
    {
        return new InstantCommand(
                () -> {
                    this.setPower(power.getAsDouble());
                }
        );
    }

    /**
     * Returns RPM
     * @return
     */
    public Double getMotorVelocity()
    {
        return shooter.getVelocity() / encoderResolution * 60.0;
    }

    private double calculatePlantOutput(double targetRPM)
    {
        return Math.signum(targetRPM * ShooterConstants.kV) * ShooterConstants.kS + targetRPM * ShooterConstants.kV;
    }

    public boolean nearTargetRPM(double targetRPM)
    {
        return (Math.abs(getMotorVelocity()-targetRPM) < ShooterConstants.RPMtolerance);
    }

    public bangRPM bang(DoubleSupplier targetRPM)
    {
        return new bangRPM(targetRPM);
    }

    public Command shooterOff()
    {
        return this.setPowerC(0.0);
    }

    public class bangRPM extends CommandBase
    {
        DoubleSupplier targetRPM = () -> 0;
        ShooterSubsystem shooterSubsystem = ShooterSubsystem.this;
        public Trigger nearTargetRPM = new Trigger(this::nearTargetRPM).and(new Trigger(this::isScheduled));
        private boolean auto = false;

        public bangRPM(DoubleSupplier targetRPM)
        {
            this.targetRPM = targetRPM;

            addRequirements(ShooterSubsystem.this);
        }

        public Command isAuto(boolean auto)
        {
            this.auto = auto;
            return this;
        }

        @Override
        public void execute() {
            if (shooterSubsystem.getMotorVelocity() < targetRPM.getAsDouble())
            {
                shooterSubsystem.setPower(1.0);
            }
            else if (shooterSubsystem.getMotorVelocity() > targetRPM.getAsDouble() + 200 && !auto)
            {
                shooterSubsystem.setPower(-1.0);
            }
            else
            {
                shooterSubsystem.setPower(0);
            }
        }

        @Override
        public void end(boolean interrupted) {
            shooterSubsystem.setPower(0.0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        public boolean nearTargetRPM()
        {
            return this.shooterSubsystem.nearTargetRPM(this.targetRPM.getAsDouble());
        }
    }
}