package org.firstinspires.ftc.teamcode.opmode.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.DebugSequential;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

import java.util.Iterator;

public class QualsCloseBase extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            super.initDebug();
        }



        waitForStart();

        Iterator<BezierCommand> it = this.pathCommands.iterator();

        CommandScheduler.getInstance().schedule(
                new DebugSequential(
                        new ScheduleCommand(this.shooterSubsystem.bang(sotm::getRPM)),
                        it.next(), // Preload 3
                        new WaitCommand(AutoConstants.spinupWait),
                        transfer(),
                        regularCycle(it), //Middle 3
                        gateCycle(it),
                        regularCycle(it), // Close 3
                        gateCycle(it),
                        gateCycle(it),
                        closeGateWhileLeaving(),
                        intake(),
                        it.next().interruptOn(() -> pinpointSubsystem.getPose().getTranslation().getY() > 60), // Middle >
                        it.next(), // Middle <
                        turretStabilize(),
                        turretStabilize(),
                        turretStabilize(),
                        transfer()
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            if (isStopRequested())
            {
                return;
            }
            update();
            telemetry.addData("Pinpoint Pose", pinpointSubsystem.getPose());
        }

//        end();
    }

    public void initializeRedPaths() {
        // First 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory1));

        // Middle 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory2));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory3));

        // Gate with modified return rotation
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory5B));

        // Close 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory6));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory7B));

        // Gate
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory5A));

        // Gate
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory5A));

        // Far 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory8));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory9));
    }
}