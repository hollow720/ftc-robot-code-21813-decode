package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

@Autonomous(name="RED Playoff 21")
public class RedPlayoffClose21 extends PlayoffCloseBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.RED;
        initialize(DriveMode.VELODRIVE, RedCloseAutoConstants.P1S, this.alliance);

        initializeRedPaths();

        super.runOpMode();
    }
}
