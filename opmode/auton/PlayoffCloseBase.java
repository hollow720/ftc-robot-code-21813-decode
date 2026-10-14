package org.firstinspires.ftc.teamcode.opmode.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.DebugSequential;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

import java.util.Iterator;

public class PlayoffCloseBase extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit())
        {
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
                        gateCycle(it),
                        gateCycle(it),
                        gateCycle(it),
                        regularCycle(it) // Close 3
                )
        );

        while (opModeIsActive() && !isStopRequested()){
            if (isStopRequested()) {

                return;
            }

            update();
            telemetry.addData("Pinpoint Pose", pinpointSubsystem.getPose());
        }

//        end();
    }

    public void initializeRedPaths()
    {
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory1));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory2));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory3));

        for (int i = 0; i < 3; i++)
        {
            this.pathCommands.add(new BezierCommand(
                    pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4));

            this.pathCommands.add(new BezierCommand(
                    pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory5A));
        }

        //Modified Gate Return Rotation
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory5B));

        //Cube robot leave last 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory6));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory7A));
    }
}
