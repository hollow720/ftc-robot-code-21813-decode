package org.firstinspires.ftc.teamcode.control.kinematics;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Translation2d;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

import java.util.LinkedList;
import java.util.List;

public class SixWheelMecanumKinematics {
    private final SimpleMatrix inverseKinematics;
    private final SimpleMatrix forwardKinematics;

    private final Translation2d fl;
    private final Translation2d ml;
    private final Translation2d bl;
    private final Translation2d fr;
    private final Translation2d mr;
    private final Translation2d br;

    /**
     * Give Translation2d in Meters.
     * @param fl
     * @param ml
     * @param bl
     * @param fr
     * @param mr
     * @param br
     */
    public SixWheelMecanumKinematics(Translation2d fl, Translation2d ml, Translation2d bl, Translation2d fr, Translation2d mr, Translation2d br)
    {
        this.fl = fl;
        this.ml = ml;
        this.bl = bl;
        this.fr = fr;
        this.mr = mr;
        this.br = br;

        inverseKinematics = new SimpleMatrix(6, 3);

        setInverseKinematics(fl, ml, bl, fr, mr, br);
        forwardKinematics = inverseKinematics.pseudoInverse(); //wippy lib uses it idk why
//        Log.i("MATRIX inverse", inverseKinematics.toString());
//        Log.i("MATRIX forward", forwardKinematics.toString());
    }

    private void setInverseKinematics(Translation2d fl, Translation2d ml, Translation2d bl, Translation2d fr, Translation2d mr, Translation2d br)
    {
        this.inverseKinematics.setRow(0, 0, 1, -1, -(fl.getX()+fl.getY()));
        this.inverseKinematics.setRow(1, 0, 1, -1, -(ml.getX()+ml.getY()));
        this.inverseKinematics.setRow(2, 0, 1, 1, bl.getX()-bl.getY());
        this.inverseKinematics.setRow(3, 0, 1, 1, fr.getX()-fr.getY());
        this.inverseKinematics.setRow(4, 0, 1, 1, mr.getX()-mr.getY());
        this.inverseKinematics.setRow(5, 0, 1, -1, -(br.getX()+br.getY()));
    }

    /**
     * Wheel Speeds in Meters/s
     *
     * 1. FL
     * 2. ML
     * 3. BL
     * 4. FR
     * 5. MR
     * 6. BR
     * @return
     */
    public List<Double> toWheelSpeeds(ChassisSpeed cs)
    {
        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(0, 0, cs.getVx(), cs.getVy(), cs.getOmega());
        SimpleMatrix wheelsVector = inverseKinematics.mult(chassisSpeedsVector);

        return List.of(
                wheelsVector.get(0, 0),
                wheelsVector.get(1, 0),
                wheelsVector.get(2, 0),
                wheelsVector.get(3, 0),
                wheelsVector.get(4, 0),
                wheelsVector.get(5, 0));
    }

    public ChassisSpeed toChassisSpeed(List<Double> wheelSpeeds)
    {
        SimpleMatrix wheelMatrix = new SimpleMatrix(6, 1);
        for (int i = 0; i < 6; i++)
        {
            wheelMatrix.setRow(i, 0, wheelSpeeds.get(i));
        }
        SimpleMatrix chassisSpeed = forwardKinematics.mult(wheelMatrix);
//        Log.i("MATRIX", chassisSpeed.toString());
        return new ChassisSpeed(
                chassisSpeed.get(0, 0),
                chassisSpeed.get(1, 0),
                chassisSpeed.get(2, 0));
    }

    /**
     * Given Radian/s, desaturate max velocity to maintain direction
     * @param wheelSpeeds
     * @return
     */
    public List<Double> deSaturateWheelSpeeds(List<Double> wheelSpeeds)
    {
        double largestAngularSpeed = wheelSpeeds.stream().mapToDouble(Math::abs).max().orElse(1.0);

        double scalar = 1.0;
        if (largestAngularSpeed > DriveConstants.maxWheelAngularVelocityRadiansPerSecond.getAsDouble())
        {
            scalar = DriveConstants.maxWheelAngularVelocityRadiansPerSecond.getAsDouble() / largestAngularSpeed;
        }

        LinkedList<Double> wheelsSpeeds = new LinkedList<>();
        for (Double speed : wheelSpeeds)
        {
//            Log.i("saturated speed", speed.toString());
            wheelsSpeeds.add(speed * scalar);
//            Log.i("desaturated speed", String.valueOf(speed * scalar));
        }

        return wheelsSpeeds;

        // Max version
//        double largestAngularSpeed = wheelSpeeds.stream()
//                .mapToDouble(a -> Math.abs(a))
//                .max().orElse(0);
//
//        double scale;
//        if (largestAngularSpeed > maxWheelAngularSpeed) {
//            scale = maxWheelAngularSpeed / largestAngularSpeed;
//        } else {
//            scale = 1.0;
//        }
//
//        return wheelSpeeds.stream()
//                .map(a -> a * scale)
//                .collect(Collectors.toList());
    }
}
