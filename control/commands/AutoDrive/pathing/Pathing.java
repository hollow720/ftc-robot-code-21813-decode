package org.firstinspires.ftc.teamcode.control.pathing;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class Pathing {
    public static Pose2d mirrorAlliancePose(Pose2d pose)
    {
        return new Pose2d(
                pose.getX(),
                -pose.getY(),
                pose.getRotation().times(-1.0)
        );
    }
}
