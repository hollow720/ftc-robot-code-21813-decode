package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;

import java.util.function.DoubleSupplier;

@Disabled
@TeleOp(name="actual", group="0")
public class trenches extends LinearOpMode {
    MecanumDriveSubsystem mecanumSubsystem;
    GamepadEx driverPad;
    DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        mecanumSubsystem = new MecanumDriveSubsystem(hardwareMap, null);
        mecanumSubsystem.setDriveModeALL(true);

        DoubleSupplier y = () -> -gamepad1.left_stick_y * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble();
        DoubleSupplier x = () -> -gamepad1.left_stick_x * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble();
        DoubleSupplier rx = () -> -gamepad1.right_stick_x * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble();
        mecanumSubsystem.setDefaultCommand(new RunCommand(() ->
        {
            mecanumSubsystem.driveRobotCentricWithVelocity(
                            new ChassisSpeed(
                                    y.getAsDouble(),
                                    x.getAsDouble(),
                                    rx.getAsDouble()));
        }, mecanumSubsystem));

        driverPad = new GamepadEx(gamepad1);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            telemetry.addData("looptime", timer.milliseconds());
            timer.reset();
            telemetry.update();
        }

        CommandScheduler.getInstance().reset();


    }
}
