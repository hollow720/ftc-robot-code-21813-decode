package org.firstinspires.ftc.teamcode.opmode.Test.sysID;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

@Disabled
@Deprecated
@TeleOp(name = "Individual Wheel Drive", group="3")
public class IndividualWheelTest extends Robot {
    public static int iterator = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.aWasPressed())
            {
                iterator++;

                if (iterator > 5)
                {
                    iterator = 0;
                }
            }

            telemetry.addData("Wheel Name", mecanumSubsystem.addressIndividual(iterator, 1));
            update();
        }
    }
}
