package org.firstinspires.ftc.teamcode.opmode.Test.sysID;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "dt kVkA Test", group="3")
//@Config
public class kVkATest extends Robot {
    public static double forwardPower = 0.05;
    public static double periodMS = 2000; //ms
    public static double increment = 0.05;
    public static double ending = 0.30;
    public List<Pair<Double, Double>> data = new ArrayList<>();
    public List<Double> cycleData = new ArrayList<>();
    public boolean forward = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(DriveMode.SIMPLEDRIVE, Alliance.RED);

        waitForStart();
        timer.reset();
        while (!isStopRequested() && opModeIsActive()) {
            if (forwardPower > ending)
            {
                int cycles = (int)(ending/increment);
                SimpleMatrix velocity = new SimpleMatrix(cycles, 2);
                SimpleMatrix applied = new SimpleMatrix(cycles, 1);

                for (int i = 0; i < cycles; i++)
                {
                    Pair<Double, Double> pair = data.get(i);
                    applied.set(i, pair.first);
                    velocity.set(i, 0, 1);
                    velocity.set(i, 1, pair.second);
                }

                //V = kS * sgn(d'(t)) + kV * d'(t)
                //V = A * K
                //But since we only have positive tuning values
                // => V = kS * 1 + kV * d'(t)
                /*

                [V1     [1, d'(t)  [kS
                 V2  =   1, d'(t)   kV]
                 V3]     1, d'(t)]

                [kS
                 kV] = A† * K

                 */
                SimpleMatrix k = velocity.pseudoInverse().mult(applied);

                telemetry.addData("kS", k.get(0, 0));
                telemetry.addData("kV", k.get(1, 0));
                telemetry.addData("collected data", data);
                telemetry.addData("-- QuasiStatic Test Completed --", "YAY!!");
                telemetry.update();
            }
            else {

                //Movement
                mecanumSubsystem.driveRobotCentricWithPower(forward ? forwardPower : -forwardPower, 0, 0);
                cycleData.add(Math.abs(pinpointSubsystem.getFieldRelativeVelocityMetric().getTranslation().getNorm()));

                //Iteration logic
                if (timer.milliseconds() > periodMS) {
                    if (forward) {
                        // First find the largest forward velocity and add to storage
                        data.add(new Pair<>(forwardPower, cycleData.stream().max(Double::compareTo).orElse(0.0)));
                        // Then clear cycle data
                        cycleData.clear();
                    } else {
                        forwardPower += increment;
                    }

                    forward = !forward;
                    timer.reset();
                    mecanumSubsystem.driveRobotCentricWithPower(0, 0, 0);
                    Thread.sleep(500);
                }

                telemetry.addData("forward power \t", forwardPower);
                telemetry.addData("forward? \t", forward);
                telemetry.addData("collected data", data);
                update();
            }
        }

        forwardPower = 0.05;
        forward = true;
    }
}
