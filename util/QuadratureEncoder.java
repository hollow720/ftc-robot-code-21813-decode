package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class QuadratureEncoder {
    final DcMotor encoderReference;
    final double startingDefOffset;

    public QuadratureEncoder(DcMotor encoderReference, boolean reverseEncoder, double startingDefOffset)
    {
        this.encoderReference = encoderReference;

        if (reverseEncoder) {
            encoderReference.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            encoderReference.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        encoderReference.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderReference.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.startingDefOffset = startingDefOffset;
    }

    public double getPositionDegrees()
    {
        return ((360.0 * (encoderReference.getCurrentPosition()/8192.0) * 1/2.0) + startingDefOffset);
    }
}
