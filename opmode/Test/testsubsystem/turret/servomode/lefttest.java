package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.servomode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.util.AxonEncoder;

@TeleOp(name="left turret test", group = "1")
public class lefttest extends LinearOpMode {

    static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        AxonEncoder leftEncoder = new AxonEncoder(hardwareMap, TurretConstants.leftEncoderConfig.getName());
        ServoImplEx left = hardwareMap.get(ServoImplEx.class, "leftTurret");

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.leftBumperWasPressed())
            {
                target -= 0.05;
            }
            if (gamepad1.rightBumperWasPressed())
            {
                target += 0.05;
            }

            telemetry.addData("Target Servo Position", left.getPosition());
            telemetry.addData("Analog Position", TurretConstants.leftEncoderConfig.normalize360Degrees(leftEncoder.getPositionDegrees()));
            left.setPosition(target);

            telemetry.update();
        }
    }
}
