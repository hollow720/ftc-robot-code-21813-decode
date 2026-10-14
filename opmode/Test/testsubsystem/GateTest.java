package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="Gate Test", group="3")
public class GateTest extends LinearOpMode {
    public static double targetPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx s = hardwareMap.get(ServoImplEx.class, "gate");

        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            s.setPosition(targetPosition);

            telemetry.addData("targetPosition", targetPosition);
            telemetry.update();
        }
    }
}
