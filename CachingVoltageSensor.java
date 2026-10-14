package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.design.TelemetrySubsystem;

@Config
public class CachingVoltageSensor extends TelemetrySubsystem {

    public static int voltageIndex = 0;
    public static boolean cached = false;
    /**
     * This is the voltage we want the robot to always try to operate at.
     */
    public static double    nominalVoltage = 12;
    public static double    voltageStabilizeClampMax = 13.1;
    public static double    voltageStabilizeClampMin = 9;
    //Yo im curious about if atp whether theres a set thres for brownout disconnects
    public static double cacheInvalidateSeconds = 0.5;

    private VoltageSensor voltageSensor;
    public double voltage = 0;
    private ElapsedTime timer = new ElapsedTime();

    public CachingVoltageSensor(HardwareMap hmap) {

        // the voltages of both hubs should be roughly equal so we want the chub sensor, which we don't know which one it is :(
        voltageSensor = hmap.getAll(VoltageSensor.class).get(voltageIndex);
        timer.reset();
    }

    @Override
    public void periodic() {
//        telemetryC.accept("measured voltage", this.getVoltage());
//        Log.i("measured voltage", this.getVoltage().toString());
//        Log.i("voltage compensation scalar", String.format("%f", this.getVoltageNormalized()));
    }

    /**
     * @return The last cached voltage measurement.
     */
    public Double getVoltage() {
        if (timer.seconds() > cacheInvalidateSeconds && cacheInvalidateSeconds >= 0) {
            timer.reset();
            clearCache();
        }

        if (!cached) {
            cached = true;
            return voltage = voltageSensor.getVoltage();
        } else {
            return voltage;
        }
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal voltage from the current voltage.
     */
    public double getVoltageNormalized() {
        //Anti Brownout Scalar
        if (getVoltage() < voltageStabilizeClampMin) //It's implied you're drawing a lot of current for it to get under this range
        { // when ur below 8V the logic level starts to die
            return 0.9 - Math.abs(voltageStabilizeClampMin - getVoltage()) * 0.1;
        }

        return nominalVoltage / MathUtil.clamp(getVoltage(), voltageStabilizeClampMin, voltageStabilizeClampMax);
    }

    public double getVoltageNormalizedShooterOnly()
    {
        return 13.1 / MathUtil.clamp(getVoltage(), voltageStabilizeClampMin, voltageStabilizeClampMax);
    }



    /**
     * Forcibly invalidates the cache.
     */
    public void clearCache() {
        cached = false;
    }

}
