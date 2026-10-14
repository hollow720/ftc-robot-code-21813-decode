package org.firstinspires.ftc.teamcode.opmode.Test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.control.commands.DirectToPointCommand;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

//@Config
@Autonomous(group="2", name="gtp test")
public class gtpTest extends Robot {

    public static double startX = 0;
    public static double startY = 0;
    public static double startWRad = 0;
    public static Pose2d startPose = new Pose2d(new Translation2d(startX, startY), new Rotation2d(startWRad));
    public static double targetX = RedCloseAutoConstants.trajectory4.getEndingPose().getX();
    public static double targetY = RedCloseAutoConstants.trajectory4.getEndingPose().getY();
    public static double targetRotDeg = RedCloseAutoConstants.trajectory4.endRot.getDegrees();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(startPose);

        DirectToPointCommand gtp = new DirectToPointCommand(mecanumSubsystem, pinpointSubsystem,
                new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRotDeg)));

        while (opModeInInit())
        {
            this.initDebug();
        }

        waitForStart();
        CommandScheduler.getInstance().schedule(gtp);
        while (!isStopRequested() && opModeIsActive())
        {
//            gtp.setTargetPose(new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRotDeg)));
            telemetry.addData("Pinpoint Position", pinpointSubsystem.getPose());
            update();
        }

        end();
    }
}
