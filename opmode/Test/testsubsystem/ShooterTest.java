package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

//@Disabled
@TeleOp(name = "Power Shooter Test", group="3")
@Config
public class ShooterTest extends Robot {
    public static double targetPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        initialize();

        DcMotorImplEx shooter = hardwareMap.get(DcMotorImplEx.class, "shooter");


        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
//            CommandScheduler.getInstance().schedule(shooterSubsystem.setPowerC(targetPower));
            shooter.setPower(targetPower);
            telemetry.addData("RPM", shooter.getVelocity());
            telemetry.update();
        }
    }
}
