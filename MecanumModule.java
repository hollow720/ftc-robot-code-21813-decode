package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.DriveConstants.ModelConstants.kS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants.ModelConstants;
import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.util.CachingVoltageSensor;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

@Config
public class MecanumModule extends TelemetrySubsystem {
    private final double WHEEL_DIAMETER;
    private final Translation2d vectorFromCenter;
    private final DcMotorImplEx motor;
    private boolean velocityControl = false;
    private double targetAngularVelocity = 0;
    public static double maxPowerSlewRate = 12;
    private SlewRateLimiter slew;
    public double cache = 0;
    public String moduleName;

    public MecanumModule(double wheelDiameter, Translation2d vectorFromCenter, DcMotorImplEx motor, String moduleName, boolean isReversed)
    {
        this.motor = motor;
        this.moduleName = moduleName;
        this.WHEEL_DIAMETER = wheelDiameter;
        this.vectorFromCenter = vectorFromCenter;
        this.slew = new SlewRateLimiter(this.maxPowerSlewRate);

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setDirection(isReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        if (velocityControl)
        {
            double power = this.getCurrentTargetLinearVelocity() * ModelConstants.kV + Math.signum(this.getCurrentTargetLinearVelocity()) * kS;
            this.setPower(power);
        }
    }

    /**
     * Sets... The power :)
     * @param power
     */
    public void setPower(double power)
    {
        double limited = slew.calculate(power);
        cache = limited;
        this.motor.setPower(cache);
    }

    public double getPower()
    {
        return this.motor.getPower();
    }

    public void setVelocityControl(boolean useVelocity)
    {
        this.velocityControl = useVelocity;
    }

    /**
     * Takes angular velocity in radian/s
     * @param angularVelocity
     */
    public void setTargetAngularVelocity(Double angularVelocity){
        this.targetAngularVelocity = angularVelocity;
    }

    public double getCurrentTargetLinearVelocity()
    {
        return this.targetAngularVelocity * (WHEEL_DIAMETER/2);
    }

    public double getCurrentTargetAngularVelocity()
    {
        return this.targetAngularVelocity;
    }

    /**
     * v_w = v_l/r
     *
     * Takes linear velocity in m/s
     *
     * @param linearVelocity
     */
    public void setTargetLinearVelocity(Double linearVelocity)
    {
        setTargetAngularVelocity(linearVelocity/(WHEEL_DIAMETER/2));
    }

    public Translation2d getVectorFromCenter()
    {
        return this.vectorFromCenter;
    }

    public String getName()
    {
        return this.moduleName;
    }
}
