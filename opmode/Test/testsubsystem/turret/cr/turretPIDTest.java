//package org.firstinspires.ftc.teamcode.opmode.Test.testsubsystem.turret.cr;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.constants.TurretConstants;
//import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SOTM;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;
//
//@Disabled
//@TeleOp(name = "Turret Subsystem PID Test", group="1")
//@Config
//public class turretPIDTest extends LinearOpMode {
//    public static double targetPosition = 15;
////    public static double kP = TurretConstants.kP_normal_turret;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Robot.telemetryC = telemetry::addData;
//
//        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, false);
//        PinpointSubsystem pinpointSubsystem = new PinpointSubsystem(hardwareMap, telemetry);
//        SOTM sotm = new SOTM(Robot.Alliance.RED, pinpointSubsystem);
//        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, Robot.Alliance.RED,
//                pinpointSubsystem, sotm);
//
//        waitForStart();
//
//        GamepadEx driver = new GamepadEx(gamepad1);
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(
//                new InstantCommand(() ->
//                {
//                    targetPosition -= 5;
//                }).andThen(turretSubsystem.setTargetPositionC(() -> targetPosition))
//        );
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(
//                new InstantCommand(() ->
//                {
//                    targetPosition += 5;
//                }).andThen(turretSubsystem.setTargetPositionC(() -> targetPosition))
//        );
//
//        while (opModeIsActive())
//        {
//
//            CommandScheduler.getInstance().run();
//            telemetry.addData("targetPosition opmode", targetPosition);
//            telemetry.update();
//        }
//
//        CommandScheduler.getInstance().reset();
//    }
//}
