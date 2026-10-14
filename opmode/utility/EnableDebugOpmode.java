package org.firstinspires.ftc.teamcode.opmode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

@Disabled
@TeleOp(group="2", name="EnableStickyDebug")
public class EnableDebugOpmode extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot.stickyDebug = true;
    }
}
