package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.DriveConstants.kinematics;
import static org.firstinspires.ftc.teamcode.constants.DriveConstants.strafeRotCompensation;

import static java.lang.Double.isNaN;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.subsystems.design.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;

import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class MecanumDriveSubsystem extends TelemetrySubsystem implements Drivetrain {
    private final MecanumModule fr;
    private final MecanumModule mr;
    private final MecanumModule br;
    private final MecanumModule fl;
    private final MecanumModule ml;
    private final MecanumModule bl;
    private final List<MecanumModule> moduleList;
    private final PinpointSubsystem pinpointSubsystem;
    public MecanumDriveSubsystem(HardwareMap hMap, PinpointSubsystem pinpointSubsystem) {
        fl = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.frontLeftR,
                hMap.get(DcMotorImplEx.class, DriveConstants.frontLeftName), DriveConstants.frontLeftName, false);
        ml = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.middleLeftR,
                hMap.get(DcMotorImplEx.class, DriveConstants.middleLeftName), DriveConstants.middleLeftName, false);
        bl = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.backLeftR,
                hMap.get(DcMotorImplEx.class, DriveConstants.backLeftName), DriveConstants.backLeftName, false);
        fr = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.frontRightR,
                hMap.get(DcMotorImplEx.class, DriveConstants.frontRightName), DriveConstants.frontRightName, true);
        mr = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.middleRightR,
                hMap.get(DcMotorImplEx.class, DriveConstants.middleRightName), DriveConstants.middleRightName, true);
        br = new MecanumModule(DriveConstants.wheelDiameter, DriveConstants.backRightR,
                hMap.get(DcMotorImplEx.class, DriveConstants.backRightName), DriveConstants.backRightName, true);

        moduleList = new LinkedList<MecanumModule>();

        moduleList.add(fl);
        moduleList.add(ml);
        moduleList.add(bl);
        moduleList.add(fr);
        moduleList.add(mr);
        moduleList.add(br);

        this.pinpointSubsystem = pinpointSubsystem;
    }

    @Override
    public void periodic() {
//        for (MecanumModule m : moduleList)
//        {
//            telemetryC.accept(m.moduleName + " target angular velo", m.getCurrentTargetAngularVelocity());
//            telemetryC.accept(m.moduleName + " actual power", m.cache);
//        }
    }

    public void driveRobotCentricWithVelocity(ChassisSpeed cs) {
        List<Double> wheelSpeeds = kinematics.toWheelSpeeds(cs);
        wheelSpeeds = kinematics.deSaturateWheelSpeeds(wheelSpeeds);
        //Wheel Speeds in Meters/s 1. FL 2. ML 3. BL 4. FR 5. MR 6. BR
        for (double w : wheelSpeeds)
        {
            if (isNaN(w))
            {
                return;
            }
        }

        fl.setTargetLinearVelocity(wheelSpeeds.get(0));
        ml.setTargetLinearVelocity(wheelSpeeds.get(1));
        bl.setTargetLinearVelocity(wheelSpeeds.get(2));
        fr.setTargetLinearVelocity(wheelSpeeds.get(3));
        mr.setTargetLinearVelocity(wheelSpeeds.get(4));
        br.setTargetLinearVelocity(wheelSpeeds.get(5));
    }

    public void driveFieldCentricWithVelocity(double vx, double vy, double vw) {
        Translation2d compensatedVelocity = new Translation2d(vx, vy).rotateBy(pinpointSubsystem.getPose().getRotation().unaryMinus());
        this.driveRobotCentricWithVelocity(
                new ChassisSpeed(
                        compensatedVelocity.getX(),
                        compensatedVelocity.getY(),
                        vw
                )
        );
    }

//    public void setCurrentAlertALL(double A) {
//        for (MecanumModule m : moduleList) {
//            m.setCurrentAlertThres(A);
//        }
//    }

    public void driveRobotCentricWithPower(double x, double y, double rx)
    {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (x + y + rx)/denominator; //x -y
        double backLeftPower = (x - y + rx)/denominator; //x y
        double frontRightPower = (x - y - rx)/denominator; //x -y
        double backRightPower = (x + y - rx)/denominator; //x -y
        double middleLeftPower = (x + y + rx)/denominator; //x -y
        double middleRightPower = (x - y - rx)/denominator; //x y

        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        fr.setPower(frontRightPower);
        br.setPower(backRightPower);
        ml.setPower(middleLeftPower);
        mr.setPower(middleRightPower);
    }

    public List<Double> getPowerList()
    {
        return this.moduleList.stream().map(mecanumModule -> mecanumModule.getPower()).collect(Collectors.toList());
    }

    public void setDriveModeALL(boolean useVelocity)
    {
        this.moduleList.stream().forEach((module) -> module.setVelocityControl(useVelocity));
    }

    public static ChassisSpeed compensateStrafeRotationDrift(ChassisSpeed cs)
    {
        return new ChassisSpeed(
                cs.getVx(),
                cs.getVy(),
                cs.getOmega() - strafeRotCompensation * cs.getVy()
        );
    }

    /**
     * DEBUG ONLY METHOD
     */
    public String addressIndividual(int index, double power)
    {
        this.moduleList.get(index).setPower(power);
        return this.moduleList.get(index).getName();
    }
}
