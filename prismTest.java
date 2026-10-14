package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name="Prism Test", group="LED")
public class prismTest extends LinearOpMode {

    GoBildaPrismDriver prism;

    PrismAnimations.Solid redSolid = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid blueSolid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid greenSolid = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.RainbowSnakes rainbow = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Blink blink = new PrismAnimations.Blink(Color.YELLOW);

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        redSolid.setBrightness(100);
        blueSolid.setBrightness(75);
        greenSolid.setBrightness(50);

        rainbow.setNumberOfSnakes(3);
        rainbow.setSnakeLength(4);
        rainbow.setSpeed(1.0f);

        blink.setBrightness(100);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Device ID", prism.getDeviceID());
        telemetry.addData("Firmware", prism.getFirmwareVersionString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                prism.clearAllAnimations();
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, redSolid);
                telemetry.addData("Pattern", "Solid Red");
            }

            if (gamepad1.b) {
                prism.clearAllAnimations();
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, blueSolid);
                telemetry.addData("Pattern", "Solid Blue");
            }

            if (gamepad1.x) {
                prism.clearAllAnimations();
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, greenSolid);
                telemetry.addData("Pattern", "Solid Green");
            }

            if (gamepad1.y) {
                prism.clearAllAnimations();
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, rainbow);
                telemetry.addData("Pattern", "Rainbow Snakes");
            }

            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("A = Red | B = Blue | X = Green");
            telemetry.addLine("Y = Rainbow | LB = Blink | RB = Off");
            telemetry.addData("FPS", prism.getCurrentFPS());
            telemetry.update();

            sleep(50);
        }
    }
}