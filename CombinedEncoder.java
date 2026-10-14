package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class CombinedEncoder {
    final AxonEncoder encoder1;
    final AxonEncoder encoder2;
    final TurretConstants.encoderConfig encoder1config;
    final TurretConstants.encoderConfig encoder2config;

    public CombinedEncoder(AxonEncoder encoder1, TurretConstants.encoderConfig encoder1config,
                           AxonEncoder encoder2, TurretConstants.encoderConfig encoder2config)
    {
        this.encoder1 = encoder1;
        this.encoder1config = encoder1config;

        this.encoder2 = encoder2;
        this.encoder2config = encoder2config;
    }

    public double getPositionDegrees()
    {
        double value1 = encoder1config.normalize360Degrees(encoder1.getPositionDegrees());
        double value2 = encoder2config.normalize360Degrees(encoder2.getPositionDegrees());

        if (value1 < 1)
        {
            return value2;
        }
        else if (value2 < 1)
        {
            return value1;
        }

        //Basically if the analog encoder disconnects the voltage output is going to be zero.

        if (encoder1config.nearDeadzone(value1))
        {
            return value2;
        }
        else if (encoder2config.nearDeadzone(value2))
        {
            return value1;
        }
        else
        {
            return value1;
        }
    }

}
