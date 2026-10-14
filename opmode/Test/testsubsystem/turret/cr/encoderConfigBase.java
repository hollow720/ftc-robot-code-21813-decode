package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.util.AxonEncoder;

@Disabled
@Config
public class encoderConfigBase extends LinearOpMode {
    public static double zeroOffset = 0.0;
    public static boolean reversed = false;
    protected String encoderName;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeInInit())
        {
            if (gamepad1.leftBumperWasPressed())
            {
                zeroOffset += 0.5;
            }
            else if (gamepad1.rightBumperWasPressed())
            {
                zeroOffset -= 0.5;
            }
            else if (gamepad1.dpadLeftWasPressed())
            {
                zeroOffset += 10;
            }
            else if (gamepad1.dpadRightWasPressed())
            {
                zeroOffset -= 10;
            }


            if (gamepad1.aWasPressed())
            {
                reversed = !reversed;
            }

            telemetry.addData("Zero Offset", zeroOffset);
            telemetry.addData("reversed", reversed);
            telemetry
                    .addLine(
                            "When Tuning the Turret here are the steps you have to do. \n" +
                            "Determine the 0 yaw position. This is relative to your wiring. \n" +
                            "As of 1/26 the zero position is going to be left of the bot facing forwards. \n" +
                            "Then play with reverse and offset, to make the encoder line up with the coordinate system. \n" +
                            "Lastly, note down the range in which the encoder jumps because of the potentiometer wrap."
                    );
            telemetry.update();
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        AxonEncoder encoder = new AxonEncoder(hardwareMap, encoderName);
        TurretConstants.encoderConfig testConfig
                = new TurretConstants.encoderConfig(
                zeroOffset,
                reversed,
                encoderName,
                10,
                10
        );

        while (opModeIsActive())
        {

            loopTimer.reset();

            telemetry.addData(encoderName + " Deg", encoder.getPositionDegrees());
            telemetry.addData(encoderName + " offset Deg", testConfig.normalize360Degrees(encoder.getPositionDegrees()));

            telemetry.addData("Loop Timer (ms)", loopTimer.milliseconds());
            telemetry.update();
        }

    }
}
