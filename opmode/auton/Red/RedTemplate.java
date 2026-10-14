package org.firstinspires.ftc.teamcode.opmode.auton.Red;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;

public class RedTemplate extends RedGlobalAutoConstants {
    // === Path Constants ===
    public static Pose2d P1S = new Pose2d(62.5, 8.0, faceOpposingHP);
    public static Pose2d P1E = new Pose2d(62.5, 61, faceOpposingHP);
    public static Bezier bezier1 = new Bezier(P1S, P1E);
    public static Trajectory trajectory1 = new Trajectory(bezier1, highSpeedMotionProfile, defaultEndingConfig, P1S, P1E);

}
