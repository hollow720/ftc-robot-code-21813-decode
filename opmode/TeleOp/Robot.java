package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.CameraConstants;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SOTM;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpoilerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public abstract class Robot extends LinearOpMode {
    CommandScheduler cs = CommandScheduler.getInstance();
    public MecanumDriveSubsystem mecanumSubsystem;
    public PinpointSubsystem pinpointSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public HoodSubsystem hoodSubsystem;
    public TransferSubsystem transferSubsystem;
    public LedSubsystem ledSubsystem;
    public TurretSubsystem turretSubsystem;
    public CameraSubsystem rightCameraSubsystem;
    public SpoilerSubsystem spoilerSubsystem;
//    public TiltSubsystem tiltSubsystem;
    public SOTM sotm;
    List<LynxModule> hubs;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime loopTimer = new ElapsedTime();

    public IMU imu;
    public CachingVoltageSensor voltageSensor;
    public static boolean stickyDebug = false;
    public static BiConsumer<String, Object> telemetryC = (s, o) -> {
        Log.e("Brotato chip you're writing to nothing", "Dummy didnt refresh access haha");
        throw new NullPointerException("Shit isnt initialized gng");
    };
    public static Runnable updateTelemetry = () -> {
        Log.e("Brotato chip you're writing to nothing", "Dummy didnt refresh access haha");
        throw new NullPointerException("Shit isnt initialized gng");
    };

    public static Supplier<BiConsumer<String, Object>> getTelemetry = () -> telemetryC;
    public static Supplier<Runnable> updateSupplier = () -> updateTelemetry;
    public Alliance alliance = Alliance.RED;


    /**
     * Call this inside of your "runOpMode" method
     */
    public void initialize(DriveMode initialDrive, Alliance alliance)
    {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryC = telemetry::addData;
        updateTelemetry = telemetry::update;

        this.alliance = alliance;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (LynxModule module : hubs) {
            module.clearBulkCache();
            module.abandonUnfinishedCommands();
        }

        timer.reset();

//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        voltageSensor = new CachingVoltageSensor(hardwareMap);
        spoilerSubsystem = new SpoilerSubsystem(hardwareMap);
        pinpointSubsystem = new PinpointSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, voltageSensor);
        sotm = new SOTM(alliance, pinpointSubsystem, telemetry, shooterSubsystem::getMotorVelocity);
        mecanumSubsystem = new MecanumDriveSubsystem(hardwareMap, pinpointSubsystem);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, false);
//        rightCameraSubsystem = new CameraSubsystem(hardwareMap, CameraConstants.rightCamera, pinpointSubsystem);
        hoodSubsystem = new HoodSubsystem(hardwareMap, sotm::getHood);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        ledSubsystem = new LedSubsystem(hardwareMap, alliance);
//        prismLedSubsystem = new PrismLedSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap, this.alliance, this.pinpointSubsystem, sotm);
//        tiltSubsystem = new TiltSubsystem(hardwareMap);

        intakeSubsystem.probablyEmpty.whenActive(
                () -> {
                    ledSubsystem.queue(
                            ledSubsystem.setArtboardC(LedSubsystem.artboard.FLASHING_RED)
                                    .alongWith(new WaitCommand(300))
                                    .andThen(ledSubsystem.setArtboardC(LedSubsystem.artboard.FULL_RED))
                    );
                }
        );

        intakeSubsystem.threeBalls.whenActive(
                () -> {
                    ledSubsystem.queue(
                            ledSubsystem.setArtboardC(LedSubsystem.artboard.FLASHPURPLE)
                                    .alongWith(new WaitCommand(300))
                                    .andThen(ledSubsystem.setArtboardC(LedSubsystem.artboard.FULLPURPLE))
                    );
                }
        );


        waitForPinpoint();

        handleDriveMode(initialDrive);
    }

    public void initialize()
    {
        this.initialize(DriveMode.VELODRIVE, Alliance.RED);
    }

    public void initialize(Alliance alliance)
    {
        this.initialize(DriveMode.VELODRIVE, alliance);
    }

    public void initialize(Pose2d startingPose)
    {
        this.initialize(DriveMode.VELODRIVE, Alliance.RED);
        pinpointSubsystem.setPositionPose(startingPose);
    }

    public void initialize(DriveMode initialDrive, Pose2d startingPose, Alliance alliance)
    {
        this.initialize(DriveMode.VELODRIVE, alliance);
        pinpointSubsystem.setPositionPose(startingPose);
    }

    public void waitForPinpoint()
    {
        ElapsedTime local = new ElapsedTime();
        local.reset();
        while (pinpointSubsystem.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
        {
            pinpointSubsystem.periodic();
            telemetry.addData("Pinpoint Device Status: ", pinpointSubsystem.getDeviceStatus());
            telemetry.addData("proof this isn't static", Math.random());
            telemetry.update();
            if (local.seconds() > 5.00)
            {
                throw new RuntimeException("twin im waiting for dis damn pinpoint for too long");
            }
        }
    }

    public void update()
    {
        if (Constants.drawRobot) {
            TelemetryPacket packet = new TelemetryPacket();
            drawRobot(packet.fieldOverlay(), pinpointSubsystem.getPose(), "blue");
            drawRobot(packet.fieldOverlay(), sotm.iterativeProjectedRobotToField(), "yellow");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        telemetry.addData("Loop time", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }

    public void end()
    {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
            hub.abandonUnfinishedCommands();
        }

        ledSubsystem.turnOff();
        CommandScheduler.getInstance().reset();
    }

    public void initDebug()
    {
        pinpointSubsystem.periodic();
        telemetry.addLine("---- INIT STATUS LOOP ----");
        telemetry.addData("Pinpoint Position", pinpointSubsystem.getPose());
        telemetry.addData("Pinpoint Calibration Status: ", pinpointSubsystem.isDoneCalibration());
        telemetry.addData("Pinpoint Device Status: ", pinpointSubsystem.getDeviceStatus());
        telemetry.addData("proof this isn't static", Math.random());

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        telemetry.update();
    }

    public void handleDriveMode(DriveMode driveMode)
    {
        switch (driveMode) {
            case VELODRIVE:
                mecanumSubsystem.setDriveModeALL(true);
                break;
            case SIMPLEDRIVE:
                mecanumSubsystem.setDriveModeALL(false);
                break;
        }
    }

    public void drawRobot(Canvas c, Pose2d t, String color) {
        final double ROBOT_RADIUS = 9;
        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
        c.setStroke(color);

        Vector2d halfv = new Vector2d(t.getRotation().getCos(), t.getRotation().getSin()).times(0.5*ROBOT_RADIUS);
        Vector2d p1 = new Vector2d(t.getX(), t.getY()).plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    public void telemetryWrite(String label, Object input)
    {
        telemetry.addData(label, input);
    }

    public enum DriveMode {
        SIMPLEDRIVE,
        VELODRIVE
    }

    public enum Alliance {
        RED,
        BLUE
    }
}
