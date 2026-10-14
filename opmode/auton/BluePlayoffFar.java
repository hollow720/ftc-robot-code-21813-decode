package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedFarAutoConstants;

@Autonomous(name="Blue Playoff Far")
public class BluePlayoffFar extends RedFarBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.BLUE;
        initialize(DriveMode.VELODRIVE, Pathing.mirrorAlliancePose(RedFarAutoConstants.P1S), this.alliance);

        this.initializeBluePaths();

        super.runOpMode();
    }
}
