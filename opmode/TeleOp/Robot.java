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
        throw new NullPointerException("Object isn't initialized");
    };
    public static Runnable updateTelemetry = () -> {
        Log.e("Brotato chip you're writing to nothing", "Dummy didnt refresh access haha");
        throw new NullPointerException("Object isn't initialized");
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

    public void initialize(DriveMode initialDrive, Pose2