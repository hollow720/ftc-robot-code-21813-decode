//package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Disabled
//@TeleOp(name="quadrature test", group = "1")
//public class quadratureTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor ref = hardwareMap.get(DcMotor.class, "intake");
//
//        ref.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ref.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        ref.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Quadrature", ref.getCurrentPosition());
//            telemetry.addData("Quadrature Divided by CPR (revolutions)", ref.getCurrentPosition()/8192.0);
//            telemetry.addData("Quadrature Divided by CPR normalized ratio and degrees", 360.0 * (ref.getCurrentPosition()/8192.0) * 1/2.0);
//            telemetry.update();
//        }
//    }
//}
