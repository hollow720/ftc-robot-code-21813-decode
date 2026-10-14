//package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//
//import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;
//
//@Config
//@Disabled
//@TeleOp(name="Manual Turret Test (FF)", group="1")
//public class TurretTestManual extends LinearOpMode {
//    CRServoImplEx turretLeft;
//    CRServoImplEx turretRight;
//    CachingVoltageSensor voltageSensor;
//    public static double power = 0.2;
//    CRServo yes;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        turretLeft = hardwareMap.get(CRServoImplEx.class, "leftTurret");
//        turretRight = hardwareMap.get(CRServoImplEx.class, "rightTurret");
//        voltageSensor = new CachingVoltageSensor(hardwareMap);
//
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested())
//        {
//            if (gamepad1.rightBumperWasPressed())
//            {
//                turretLeft.setPower(power);
//                turretRight.setPower(power);
//            }
//            else if (gamepad1.leftBumperWasPressed())
//            {
//                turretLeft.setPower(-power);
//                turretRight.setPower(-power);
//            }
//            else if (gamepad1.aWasPressed())
//            {
//                turretLeft.setPower(0);
//                turretRight.setPower(0);
//            }
//
//
//            telemetry.addData("power", power);
//            telemetry.update();
//        }
//    }
//}
