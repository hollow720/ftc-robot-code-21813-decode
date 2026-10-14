package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

@Config
@TeleOp(name="Hood Reset (Full Hood Extension)", group="3")
public class HoodTest extends LinearOpMode {
    public static double targetPosition = ShooterConstants.fullExtensionHoodPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx s = hardwareMap.get(ServoImplEx.class, "hood");
        s.setPwmEnable();

        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            s.setPosition(targetPosition);

            telemetry.addData("targetPosition", targetPosition);
            telemetry.update();
        }
    }
}
