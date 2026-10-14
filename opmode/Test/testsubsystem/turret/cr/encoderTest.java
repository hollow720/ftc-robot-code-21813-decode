package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AxonEncoder;

@Config
@TeleOp(name="encoderTest",group = "1")
@Disabled
public class encoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTimer = new ElapsedTime();

        AxonEncoder leftEncoder = new AxonEncoder(hardwareMap, "leftEncoder");
        AxonEncoder rightEncoder = new AxonEncoder(hardwareMap, "rightEncoder");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {

            loopTimer.reset();


            telemetry.addData("leftEncoder Deg", leftEncoder.getPositionDegrees());
            telemetry.addData("rightEncoder Deg", rightEncoder.getPositionDegrees());

            telemetry.addData("Loop Timer (ms)", loopTimer.milliseconds());
            telemetry.update();
        }

    }
}
