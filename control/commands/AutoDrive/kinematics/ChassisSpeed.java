package org.firstinspires.ftc.teamcode.control.kinematics;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class ChassisSpeed {
    private double vxMetersPerSecond = 0;
    private double vyMetersPerSecond = 0;
    /**
        Rotation is positive in the CLOCKWISE direction
     */
    private double omegaRadiansPerSecond = 0;

    /**
     * NWU Coordinate System
     *
     * Rotation is positive in the CLOCKWISE direction
     * @param vx
     * @param vy
     * @param omega
     */
    public ChassisSpeed(double vx, double vy, double omega){
        this.vxMetersPerSecond = vx;
        this.vyMetersPerSecond = vy;
        this.omegaRadiansPerSecond = omega;
    }

    public ChassisSpeed()
    {
        this(0, 0, 0);
    }

    public double getVx()
    {
        return this.vxMetersPerSecond;
    }

    public double getVy()
    {
        return this.vyMetersPerSecond;
    }

    public double getOmega()
    {
        return this.omegaRadiansPerSecond;
    }

    public ChassisSpeed speedClampDecorator(double xClamp, double yClamp, double wClamp)
    {
        this.vxMetersPerSecond = MathUtil.clamp(vxMetersPerSecond, -xClamp, xClamp);
        this.vyMetersPerSecond = MathUtil.clamp(vxMetersPerSecond, -yClamp, yClamp);
        this.omegaRadiansPerSecond = MathUtil.clamp(omegaRadiansPerSecond, -wClamp, wClamp);

        return this;
    }

    public ChassisSpeed add(ChassisSpeed speed2)
    {
        return new ChassisSpeed(
                this.vxMetersPerSecond + speed2.vxMetersPerSecond,
                this.vyMetersPerSecond + speed2.vyMetersPerSecond,
                this.omegaRadiansPerSecond + speed2.omegaRadiansPerSecond
        );
    }

    public ChassisSpeed minus(ChassisSpeed speed2)
    {
        return new ChassisSpeed(
                this.vxMetersPerSecond - speed2.vxMetersPerSecond,
                this.vyMetersPerSecond - speed2.vyMetersPerSecond,
                this.omegaRadiansPerSecond - speed2.omegaRadiansPerSecond
        );
    }
}
