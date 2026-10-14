package org.firstinspires.ftc.teamcode.opmode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Blue TeleOp", group="0")
public class BlueTeleOp extends TeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
