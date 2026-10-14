package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

@Autonomous(name="RED Quals 21")
public class RedQualsClose21 extends QualsCloseBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.RED;
        initialize(DriveMode.VELODRIVE, RedCloseAutoConstants.P1S, this.alliance);

        this.initializeRedPaths();

        super.runOpMode();
    }
}
