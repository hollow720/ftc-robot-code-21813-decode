package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.RelocalizationConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoGoalCloseCommand;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoGoalFarCommand;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoBackCloseCommand;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoBackFarCommand;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoGateCommand;
import org.firstinspires.ftc.teamcode.control.commands.AutoDrive.AutoHumanPlayerCommand;
import org.firstinspires.ftc.teamcode.control.commands.BezierCommand;
import org.firstinspires.ftc.teamcode.control.commands.DirectToPointCommand;
import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Pathing;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedCloseAutoConstants;
import org.firstinspires.ftc.teamcode.opmode.auton.Red.RedGlobalAutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Debug (RED) TeleOp", group="0")
public class TeleOp extends Robot {
    private GamepadEx driverPad;
    private GamepadEx operatorPad;

    public boolean slowDrive = false;
    Trigger driverActive = new Trigger(() -> false);

    public static boolean tuningFat = false;
    public static boolean tuningReg = false;

    public static double tuneableRPM = 0;
    public static double tuneableHood = ShooterConstants.retractedHoodPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(this.alliance); //Default is RED

        driverPad = new GamepadEx(gamepad1);
        operatorPad = new GamepadEx(gamepad2);

        while (opModeInInit())
        {
            cs.run();
            super.initDebug();
        }

        if (isStopRequested())
        {
            stop();
        }

        driverActive = new Trigger(() -> (Math.abs(gamepad1.left_stick_x) > 0.3) || (Math.abs(gamepad1.left_stick_y) > 0.3) || (Math.abs(gamepad1.right_stick_x) > 0.3));

        DoubleSupplier y = () -> -gamepad1.left_stick_y * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble() * (slowDrive ? 0.3 : 1);
        DoubleSupplier x = () -> -gamepad1.left_stick_x * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble()  * (slowDrive ? 0.3 : 1) * 1.1;
        DoubleSupplier rx = () -> -Util.lowerDeadband(
                Util.cubicJoystickCurve(gamepad1.right_stick_x),
                0.01)
                * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble() *
                (slowDrive ? 0.3 : 1);

        mecanumSubsystem.setDefaultCommand(new RunCommand(() ->
        {
            mecanumSubsystem.driveRobotCentricWithVelocity(
                    new ChassisSpeed(y.getAsDouble(),
                            x.getAsDouble(),
                            rx.getAsDouble()));
        }, mecanumSubsystem));

        configureBindings();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Selected Alliance", this.alliance);
            telemetry.addData("Pinpoint Position", pinpointSubsystem.getPose());
//            telemetry.addData("Number of tags in Camera 1", rightCameraSubsystem.numberOfDetectedTags());
            telemetry.addData("projected inches from goal", this.sotm.projectedInchesFromGoalSOTM());
            telemetry.addData("Operator Offset (Deg)", turretSubsystem.getOperatorOffset());

//            telemetry.addData("IMU", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            if (isStopRequested())
            {
                stop();
            }

            update();
        }

        end();
    }

    public void configureBindings()
    {
        // ==== Driver Bindings ====
        new Trigger(() -> gamepad1.share).whenActive(new InstantCommand(() -> {
            this.slowDrive = !this.slowDrive;
        }));

        new Trigger(() -> gamepad1.options).whenActive(
                relocalizeOperatorGoal()
        );

        new Trigger(() -> driverPad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2).whenActive(
                intakeSubsystem.intake()
        ).whenInactive(
                intakeSubsystem.stop()
        );
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenActive(
                intakeSubsystem.eject()
        ).whenInactive(
                intakeSubsystem.stop()
        );

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenActive(
                pathThenGateCommand().interruptOn(() -> driverActive.get())
        );

//        driverPad.getGamepadButton(GamepadKeys.Button.B).whenActive(
//                rightCameraSubsystem.getLocalizeCommand(pinpointSubsystem)
//        );


        // ==== SHARED controls ====
        ShooterSubsystem.bangRPM bang;
        /**
         * uncomment these for tuning
         */
        if (tuningFat) {
            bang = this.shooterSubsystem.bang(() -> tuneableRPM);
            CommandScheduler.getInstance().schedule(hoodSubsystem.setHoodCContinuous(
                    () -> this.tuneableHood + Math.abs(this.tuneableRPM - shooterSubsystem.getMotorVelocity())/3000.0
            ));
        }
        else if (tuningReg)
        {
            bang = this.shooterSubsystem.bang(() -> tuneableRPM);
            CommandScheduler.getInstance().schedule(hoodSubsystem.setHoodCContinuous(
                    () -> this.tuneableHood
            ));
        }
        else {
            bang = this.shooterSubsystem.bang(sotm::getRPM);
        }

        operatorPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenActive(
                bang,
                shooterSubsystem.shooterOff()
        );

        driverPad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenActive(
                bang,
                shooterSubsystem.shooterOff()
        );

        bang.nearTargetRPM.whenActive(new SequentialCommandGroup(
                new InstantCommand(() -> gamepad1.rumble(0.8, 0.8, 50)),
                new InstantCommand(() -> gamepad2.rumble(0.8, 0.8, 50)))
        );

        driverPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(
                intakeSubsystem.openLoop(1.0).alongWith(transferSubsystem.transferC())
        ).whenInactive(transferSubsystem.stopTransferC().alongWith(intakeSubsystem.stop()));

        operatorPad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(
                intakeSubsystem.openLoop(0.6).alongWith(transferSubsystem.transferC())
        ).whenInactive(transferSubsystem.stopTransferC().alongWith(intakeSubsystem.stop()));


        // ==== OPERATOR CONTROLS ====
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenActive(
                turretSubsystem.operatorOffsetCommand(2)
        );
        operatorPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenActive(
                turretSubsystem.operatorOffsetCommand(-2)
        );

        operatorPad.getGamepadButton(GamepadKeys.Button.B).whenActive(
                relocalizeOperatorGoal()
        );

//        operatorPad.getGamepadButton(GamepadKeys.Button.A).whenActive(
//                rightCameraSubsystem.getLocalizeCommand(pinpointSubsystem)
//        );
    }

    /**
     *
     *       _=====_                               _=====_
     *      / _____ \                             / _____ \
     *    +.-'_____'-.---------------------------.-'_____'-.+
     *   /   |     |  '.        S O N Y        .'  |     |   \
     *  / ___| /|\ |___ \                     / ___|  Y  |___ \
     * / |      |      | ;  __           _   ; |             | ;
     * | | <---   ---> | | |__|         |_:> | | X         B | |
     * | |___   |   ___| ;SHARE       OPTIONS ; |___       ___| ;
     * |\    | \|/ |    /  _     ___      _   \    |  A  |    /|
     * | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
     * |  '-.______.-' /       \      /       \  '-._____.-'   |
     * |               |       |------|       |                |
     * |              /\       /      \       /\               |
     * |             /  '.___.'        '.___.'  \              |
     * |            /                            \             |
     *  \          /                              \           /
     *   \________/                                \_________/
     *                     PS2 CONTROLLER
     *
     *
     *
     */

    public Command gateAlignCommand()
    {
        if (this.alliance.equals(Alliance.RED))
        {
            return new DirectToPointCommand(mecanumSubsystem, pinpointSubsystem,
                            RedGlobalAutoConstants.gateEnd);
        }
        else {
            return new DirectToPointCommand(mecanumSubsystem, pinpointSubsystem,
                            Pathing.mirrorAlliancePose(RedGlobalAutoConstants.gateEnd));
        }
    }

    public Command pathThenGateCommand()
    {
        BezierCommand path = new BezierCommand(
                pinpointSubsystem, mecanumSubsystem, RedCloseAutoConstants.trajectory4);

        if (this.alliance.equals(Alliance.BLUE))
        {
            path = path.invertAcrossAlliance();
        }

        return path.andThen(gateAlignCommand());
    }

    public Command relocalizeOperatorGoal()
    {
        Pose2d pose;

        if (this.alliance.equals(Alliance.RED))
        {
            pose = RelocalizationConstants.redGoalRelocalizationPose;
        }
        else {
            pose = Pathing.mirrorAlliancePose(RelocalizationConstants.redGoalRelocalizationPose);
        }

        return new InstantCommand(
                () -> {
                    pinpointSubsystem.setPositionPose(pose);
                }
        ).andThen(
                new InstantCommand(
                        () -> {
                            gamepad1.rumble(300);
                        }
                )
        );
    }
}