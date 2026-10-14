package org.firstinspires.ftc.teamcode.opmode.Test.benchmarking;

import static org.firstinspires.ftc.teamcode.util.CachingVoltageSensor.voltageIndex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
@Disabled
@TeleOp(name="volttest", group="2")
public class voltwrite extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltageSensor = hardwareMap.getAll(VoltageSensor.class).get(voltageIndex);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while(opModeIsActive())
        {
            timer.reset();
            telemetry.addData("voltage", voltageSensor.getVoltage());
            telemetry.addData("time", timer.milliseconds());
            telemetry.update();
        }


    }
}
