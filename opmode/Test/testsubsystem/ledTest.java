package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LedSubsystem;

@TeleOp(name = "led test", group="3")
@Config
@Disabled
public class ledTest extends Robot {
    ElapsedTime ledTimer = new ElapsedTime();
//    private NeoDriver tempSensor;
    LedSubsystem ledSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
//        tempSensor = hardwareMap.get(NeoDriver.class, "NeoDriver");
        ledSubsystem = new LedSubsystem(hardwareMap, alliance);

        waitForStart();

        ledTimer.reset();

//        tempSensor.writeColorAll((byte)255, (byte)0, (byte)0);

        telemetry.addData("time", ledTimer.milliseconds());
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a)
            {
            }
            if (gamepad1.b)
            {
            }

            update();
        }
    }
}
