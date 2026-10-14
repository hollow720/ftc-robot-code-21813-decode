package org.firstinspires.ftc.teamcode.opmode.auton.Red;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.constants.PathingConstants;
import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;

public class RedFarAutoConstants extends RedGlobalAutoConstants {
    public static Pose2d shootingFarPos = new Pose2d(60.0, 10.0, faceOpposingHP);
    //Forward Path 1
    public static Pose2d P1S = new Pose2d(62.5, 8.0, faceOpposingHP);
    public static Translation2d P1C1 = new Translation2d(30.0, 10.0);
    public static Translation2d P1C2 = new Translation2d(30.0, 10.0);
    public static Pose2d P1E = new Pose2d(30.0, 70, faceOpposingHP);
    public static Bezier bezier1 = new Bezier(P1S, P1C1, P1C2, P1E);
    public static Trajectory trajectory1 = new Trajectory(bezier1, highSpeedMotionProfile, defaultEndingConfig, P1S, P1E);

    // Backward Path 2
    public static Pose2d P2S = P1E;
    public static Translation2d P2C1 = P1C2;
    public static Translation2d P2C2 = P1C1;
    public static Pose2d P2E = shootingFarPos;
    public static Bezier bezier2 = new Bezier(P2S, P2C1, P2C2, P2E);
    public static Trajectory trajectory2 = new Trajectory(bezier2, highSpeedMotionProfile, defaultEndingConfig, P2S, P2E);


    //Forward Path 3
    public static Pose2d P3S = trajectory2.getEndingPose();
    public static Pose2d P3E = new Pose2d(62.5, 62, faceOpposingHP);
    public static Bezier bezier3 = new Bezier(P3S, P3E);
    public static Trajectory trajectory3 = new Trajectory(bezier3, highSpeedMotionProfile, defaultEndingConfig, P3S.getRotation(), P3E.getRotation());

    //Backward Path 4
    public static Trajectory trajectory4 = trajectory3.reversePath();

}
