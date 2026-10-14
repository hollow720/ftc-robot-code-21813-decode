package org.firstinspires.ftc.teamcode.opmode.utility;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.List;

@TeleOp(group="2", name="ResetAllOpmode")
@Disabled
//@Config
public class ResetAllOpmode extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        ledSubsystem.turnOff();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.clearBulkCache();
            module.abandonUnfinishedCommands();
        }

        CommandScheduler.getInstance().reset();

        while (opModeInInit())
        {
            super.initDebug();
        }

        end();
    }
}