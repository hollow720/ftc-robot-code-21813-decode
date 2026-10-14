package org.firstinspires.ftc.teamcode.opmode.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.DebugSequential;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedFarAutoConstants;

import java.util.Iterator;

@Autonomous(name="Red 15 partner")
public class RedFarBase extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit())
        {
            super.initDebug();
        }

        if (isStopRequested())
        {
            stop();
        }

        waitForStart();

        Iterator<BezierCommand> it = this.pathCommands.iterator();

        CommandScheduler.getInstance().schedule(
                new DebugSequential(
                        new ScheduleCommand(this.shooterSubsystem.bang(sotm::getRPM)),
                        new WaitCommand(4000),
                        transfer(),
                        regularCycle(it), //Far 3
                        regularCycle(it), // Close 3
                        regularCycle(it),
                        regularCycle(it),
                        regularCycle(it),
                        regularCycle(it)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            update();
            telemetry.addData("Pinpoint Pose", pinpointSubsystem.getPose());
        }

        end();
    }


    public void initializeRedPaths() {
        // Far 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory1));

        // return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory2));

        //Hp 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        //Hp 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        //Hp 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        //Hp 3
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));

        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory3));

        //Return
        this.pathCommands.add(new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedFarAutoConstants.trajectory4));
    }
}
