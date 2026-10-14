package org.firstinspires.ftc.teamcode.opmode.auton;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.DirectToPointCommand;
import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

public abstract class AutoBase extends Robot {
    List<BezierCommand> pathCommands = new LinkedList<>();

    Command transfer()
    {
        return new WaitCommand(400).alongWith(
                intakeSubsystem.openLoop(1.0).alongWith(transferSubsystem.transferC())
        );
    }

    Command slowTransfer()
    {
        return intakeSubsystem.openLoop(0.9).alongWith(transferSubsystem.transferC());
    }

    Command stopTransfer()
    {
        return transferSubsystem.stopTransferC().alongWith(intakeSubsystem.stop());
    }

    Command intake()
    {
        return intakeSubsystem.intake();
    }

    Command hold()
    {
        return intakeSubsystem.hold();
    }

    Command turretWait()
    {
        return turretSubsystem.standardTurretWait();
    }

    Command turretStabilize()
    {
        return new WaitCommand(200);
    }

    Command waitForRolling()
    {
        return new WaitCommand(1600);
    }

    Command closeGateWhileLeaving()
    {
        return new ScheduleCommand(
                new WaitCommand(300).andThen(transferSubsystem.stopTransferC())
        );
    }

    Command stopIntakeStallingWhileLeaving()
    {
        return new ScheduleCommand(
                new WaitCommand(300).andThen(intakeSubsystem.hold())
        );
    }

    public void initializeBluePaths()
    {
        this.initializeRedPaths();
        for (ListIterator<BezierCommand> it = pathCommands.listIterator(); it.hasNext(); ) {
            BezierCommand cmd = it.next();
            it.set(cmd.invertAcrossAlliance());
        }
    }

    public void initializeRedPaths()
    {

    }

    public Command gateAlignCommand()
    {
        if (this.alliance.equals(Alliance.RED))
        {
            return new ParallelDeadlineGroup(
                    waitForRolling(),
                    new DirectToPointCommand(mecanumSubsystem, pinpointSubsystem,
                            RedGlobalAutoConstants.gateEnd)
            );
        }
        else {
            return new ParallelDeadlineGroup(
                    waitForRolling(),
                    new DirectToPointCommand(mecanumSubsystem, pinpointSubsystem,
                            Pathing.mirrorAlliancePose(RedGlobalAutoConstants.gateEnd))
            );
        }
    }

    public SequentialCommandGroup gateCycle(Iterator<BezierCommand> it)
    {
        return new SequentialCommandGroup(
                closeGateWhileLeaving(),
                intake(),
                it.next(), // Gate >
                gateAlignCommand(),
                stopIntakeStallingWhileLeaving(),
                it.next(), // Gate <
                turretStabilize(),
                transfer());
    }

    public SequentialCommandGroup regularCycle(Iterator<BezierCommand> it)
    {
        return new SequentialCommandGroup(
                closeGateWhileLeaving(),
                intake(),
                it.next().interruptOn(() -> pinpointSubsystem.getPose().getTranslation().getY() > 57), // Middle >
                it.next(), // Middle <
                turretStabilize(),
                transfer()
        );
    }
}
