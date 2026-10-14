package org.firstinspires.ftc.teamcode.opmode.Test.testdrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.util.Util;

@Deprecated
@Disabled
@TeleOp(group="1", name="Field Centric _Test_ Drive")
public class FieldCentricTest extends Robot {
    public static double forwardClamp = .5;
    public static double strafeClamp = .5;
    public static double rotClamp = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeInInit())
        {
            super.initDebug();
        }

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // Read from volatile memory
            double y = -gamepad1.left_stick_y; //Reversed because joystick's on controller's have Y inverted
            double x = -gamepad1.left_stick_x; //Reversed because XYZ mapping to NWU reverses
            double rx = -gamepad1.right_stick_x; //Reversed because rotation in Configuration Space is CCW

            double curvedJoystickY = Util.clamp(strafeClamp, -strafeClamp, y);
            double curvedJoystickX = Util.clamp(forwardClamp, -forwardClamp, x);
            double curvedJoystickRX = Util.clamp(rotClamp, -rotClamp, rx);

            mecanumSubsystem.driveFieldCentricWithVelocity(
                    curvedJoystickY * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble(),
                    curvedJoystickX * DriveConstants.maxTranslationalVelocityMetersPerSecond.getAsDouble(),
                    curvedJoystickRX * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble());

            telemetry.addData("Target X velocity", curvedJoystickY);
            telemetry.addData("Target Y velocity", curvedJoystickX);
            telemetry.addData("Target Angular velocity (CCW)", curvedJoystickRX);
            telemetry.addData("debug", curvedJoystickRX * DriveConstants.maxRotationalVelocityRadiansPerSecond.getAsDouble());
            telemetry.addData("intake power", gamepad1.left_trigger - gamepad1.right_trigger);

            if (gamepad1.xWasPressed())
            {
                stickyDebug = true;
            }

            update();
        }

        end();
    }
}