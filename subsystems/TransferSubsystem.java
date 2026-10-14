package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

public class TransferSubsystem extends TelemetrySubsystem {
    private ServoImplEx gate;
    private CRServoImplEx leftTransfer;
    private CRServoImplEx rightTransfer;

    public TransferSubsystem(HardwareMap hMap)
    {
        gate = hMap.get(ServoImplEx.class, "gate");
        leftTransfer = hMap.get(CRServoImplEx.class, "leftTransfer");
        rightTransfer = hMap.get(CRServoImplEx.class, "rightTransfer");

        rightTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        this.closeGate();
    }

    private void openGate()
    {
        this.gate.setPosition(TransferConstants.openGatePos);
    }

    private void closeGate()
    {
        this.gate.setPosition(TransferConstants.closedGatePos);
    }

    private void setPower(double power)
    {
        leftTransfer.setPower(power);
        rightTransfer.setPower(power);
    }

    public Command setPowerC(double power){
        return new InstantCommand(
                () -> {
                    this.setPower(power);
                }
        );
    }

    public Command openGateC()
    {
        return new InstantCommand(() -> {
            this.openGate();
        }, this);
    }

    public Command transferC()
    {
        return new SequentialCommandGroup(
                openGateC(),
                setPowerC(1.0)
        );
    }

    public Command stopTransferC()
    {
        return new SequentialCommandGroup(
                this.setPowerC(0.0),
                this.closeGateC()
        );
    }

    public Command closeGateC()
    {
        return new InstantCommand(() -> {
            this.closeGate();
        });
    }

    @Override
    public void periodic() {

    }
}