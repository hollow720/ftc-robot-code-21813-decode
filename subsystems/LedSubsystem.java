package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.drivers.NeoDriver;

import java.util.LinkedList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class LedSubsystem extends TelemetrySubsystem {
    ServoImplEx prism;
    LinkedList<Command> queuedCommands = new LinkedList<>();

    public LedSubsystem(HardwareMap hardwareMap, Robot.Alliance alliance)
    {
        prism = hardwareMap.get(ServoImplEx.class,"prismLED");

        prism.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));

        if (alliance.equals(Robot.Alliance.RED))
        {
            this.setArtboard(artboard.RED_SCAN);
        }
        else {
            this.setArtboard(artboard.BLUE_SCAN);
        }
    }

    @Override
    public void periodic() {
        if (queuedCommands.size() > 0)
        {
            CommandScheduler.getInstance().schedule(
                    queuedCommands.getFirst()
            );

            queuedCommands.removeFirst();
        }
    }

    public Command turnOff()
    {
        return new InstantCommand(
                () -> {
                    this.setArtboard(artboard.OFF);
                }
        );
    }

    public void queue(Command command)
    {
        this.queuedCommands.add(command);
    }

    public Command setArtboardC(artboard board)
    {
        return new InstantCommand(
                () -> {
                    this.setArtboard(board);
                }
        );
    }

    private void setArtboard(artboard board) {
        prism.setPosition(
                (board.getValue() - 500.0) / 2000.0
        );
    }

    public enum artboard {
        OFF (510),
        FLASHING_RED (515),
        RED_SCAN (525),
        BLUE_SCAN (535),
        FULL_RED (545),
        TWOTHIRDSPURPLE (555),
        FLASHPURPLE (565),
        FULLPURPLE (575);

        private final int value;

        artboard(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }


}
