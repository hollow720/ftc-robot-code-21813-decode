package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

import java.util.function.DoubleSupplier;

public class TurretSubsystem extends TelemetrySubsystem {
    ServoImplEx leftTurretServo;
    ServoImplEx rightTurretServo;
    final Command defaultTurretCommand;
    private double robotRelativeTargetPosition = 0;
    public static double turretOffset = 196; //ccw
    public static double minRangeDeg = 18.0;
    public static double maxRangeDeg = 337;
    private double operatorOffset = 0;

    private boolean togglePwm = false;

    public static double secPer60Deg = 0.090;
    Robot.Alliance alliance;

    public TurretSubsystem(HardwareMap hardwareMap, Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, SOTM sotm)
    {
        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurret");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurret");

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));

        this.defaultTurretCommand = new defaultTurretCommand(alliance, pinpointSubsystem, sotm);
        this.setDefaultCommand(defaultTurretCommand);

        this.alliance = alliance;
    }

    public Command lockGivenTargetPose(Pose2d pose)
    {
        Pose2d fieldToGoal = this.getGoalPose();
        Pose2d projectedRobotToField = pose;

        Pose2d robotToGoal = fieldToGoal.relativeTo(projectedRobotToField);
        Rotation2d robotTurnToGoal = new Rotation2d(robotToGoal.getX(), robotToGoal.getY());

        return new ScheduleCommand(
                new WaitCommand(3000).alongWith(
                        setTargetPositionC(
                                robotTurnToGoal.getDegrees()
                        )
                )
        );
    }

    public Pose2d getGoalPose()
    {
        switch (alliance)
        {
            case RED:
                return TurretConstants.redGoal;
            case BLUE:
                return TurretConstants.blueGoal;
        }

        return TurretConstants.redGoal;
    }

    @Override
    public void periodic() {
        //First offset and zero in 360 space
        double zeroedTargetDegrees = this.normalizeRange(this.robotRelativeTargetPosition + turretOffset + operatorOffset);

        //Then remove [355, 360] because axon inherently doesnt have
        double zeroedTargetDegreesClamped = Math.min(zeroedTargetDegrees, 355.0);

        //Lastly convert from Degrees (that are clamped) to Servo Space
        double target = normalizeToServoSpace(zeroedTargetDegreesClamped);
        telemetryC.accept("target turret", target);

        leftTurretServo.setPosition(target);
        rightTurretServo.setPosition(target);

        telemetryC.accept("Operator Offset", this.operatorOffset);
    }

    public WaitCommand standardTurretWait()
    {
        return new WaitCommand((long)secPer60Deg);
    }

    public void togglePwm()
    {
        if (!togglePwm)
        {
            leftTurretServo.setPwmDisable();
            rightTurretServo.setPwmDisable();
            return;
        }

        leftTurretServo.setPwmEnable();
        rightTurretServo.setPwmEnable();
    }


    /**
     * rn js convert deg to radians
     * @param robotSpaceDegrees
     * @return
     */
    public double normalizeToServoSpace(double robotSpaceDegrees)
    {
        double servoPos = robotSpaceDegrees/(355.0);
        servoPos = Math.min(maxRangeDeg/355.0, servoPos);
        servoPos = Math.max(minRangeDeg/355.0, servoPos);

        return servoPos;
    }


    /**
     * Normalizes range to [0, 360] degrees
     * @param degrees
     * @return
     */
    public double normalizeRange(double degrees)
    {
        return ((degrees % 360) + 360) % 360;
    }

    private void setRobotRelativeTargetPosition(double robotRelativeTargetPosition)
    {
        this.robotRelativeTargetPosition = robotRelativeTargetPosition;
    }

    private void addOperatorOffset(double deg)
    {
        this.operatorOffset += deg;
    }

    public double getOperatorOffset()
    {
        return this.operatorOffset;
    }

    public InstantCommand operatorOffsetCommand(double deg)
    {
        return new InstantCommand(
                () -> {
                    this.addOperatorOffset(deg);
                },
                this
        );
    }

    public InstantCommand togglePwmC()
    {
        return new InstantCommand(
                () -> {
                    this.togglePwm();
                }
        );
    }

    public InstantCommand setTargetPositionC(double targetPosition)
    {
        return new InstantCommand(
                () ->{
                    this.setRobotRelativeTargetPosition(targetPosition);
                }
        ,this);
    }

    public InstantCommand setTargetPositionC(DoubleSupplier targetPosition)
    {
        return new InstantCommand(
                () ->{
                    this.setRobotRelativeTargetPosition(targetPosition.getAsDouble());
                }
                ,this);
    }


    public Command requireCommand()
    {
        return new RunCommand(() -> {}, this);
    }

    public class defaultTurretCommand extends CommandBase
    {
        final Robot.Alliance alliance;
        final PinpointSubsystem pinpointSubsystem;
        Pose2d goalPose = new Pose2d();
        final SOTM sotm;

        public defaultTurretCommand(Robot.Alliance alliance, PinpointSubsystem pinpointSubsystem, SOTM sotm)
        {
            this.alliance = alliance;
            this.pinpointSubsystem = pinpointSubsystem;
            this.sotm = sotm;

            switch (alliance)
            {
                case RED:
                    goalPose = TurretConstants.redGoal;
                    break;
                case BLUE:
                    goalPose = TurretConstants.blueGoal;
                    break;
            }

            addRequirements(TurretSubsystem.this);
        }

        @Override
        public void execute() {
            Rotation2d robotTurnToGoal = sotm.projectedRobotTurnToGoalSOTM();

            telemetryC.accept("Target Default Command (robot space)", robotTurnToGoal.getDegrees());
            TurretSubsystem.this.robotRelativeTargetPosition = robotTurnToGoal.getDegrees();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
