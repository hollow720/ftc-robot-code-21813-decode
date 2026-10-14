package org.firstinspires.ftc.teamcode.opmode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red TeleOp", group="0")
public class RedTeleOp extends TeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        this.alliance = Alliance.RED;
        super.runOpMode();
    }
}
