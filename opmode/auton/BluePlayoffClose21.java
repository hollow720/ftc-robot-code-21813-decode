package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;

@Autonomous(name="BLUE Playoff 21")
public class BluePlayoffClose21 extends PlayoffCloseBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.BLUE;
        initialize(DriveMode.VELODRIVE, Pathing.mirrorAlliancePose(RedCloseAutoConstants.P1S), this.alliance);

        this.initializeBluePaths();

        super.runOpMode();
    }
}
