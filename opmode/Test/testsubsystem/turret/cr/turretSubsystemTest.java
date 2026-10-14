//package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;
//
//@Disabled
//@TeleOp(name = "Turret Subsystem Encoder Test", group="1")
//public class turretSubsystemTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, false);
////        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, Robot.Alliance.RED, new PinpointSubsystem(hardwareMap, telemetry), () -> 0);
//
//        waitForStart();
//
//        ElapsedTime timer = new ElapsedTime();
//
//        while (opModeIsActive())
//        {
//            timer.reset();
////            telemetry.addData("Turret Position Combined", turretSubsystem.getCombinedPosition());
////            telemetry.addData("Turret Position Quadrature", turretSubsystem.getQuadraturePosition());
//            telemetry.addData("loop time (ms)", timer.milliseconds());
//            telemetry.update();
//        }
//    }
//}
