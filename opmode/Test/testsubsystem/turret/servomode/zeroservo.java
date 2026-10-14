package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.servomode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="zeroservo", group="1")
public class zeroservo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx left = hardwareMap.get(ServoImplEx.class, "leftTurret");
        ServoImplEx right = hardwareMap.get(ServoImplEx.class, "rightTurret");

        waitForStart();

        while(opModeIsActive())
        {
            left.setPosition(0);
            right.setPosition(0);

            telemetry.update();
        }
    }
}
