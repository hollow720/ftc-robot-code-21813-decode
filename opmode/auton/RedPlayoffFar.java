package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedFarAutoConstants;

@Autonomous(name="RED Playoff Far")
public class RedPlayoffFar extends RedFarBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.RED;
        initialize(DriveMode.VELODRIVE, RedFarAutoConstants.P1S, this.alliance);

        this.initializeRedPaths();

        super.runOpMode();
    }
}
