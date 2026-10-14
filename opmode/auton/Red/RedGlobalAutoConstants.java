package org.firstinspires.ftc.teamcode.opmode.auton.Red;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import static org.firstinspires.ftc.teamcode.constants.PathingConstants.*;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;

public class RedGlobalAutoConstants {

    public static Pose2d instaTurn(Pose2d ending, Pose2d oldEnding)
    {
        return new Pose2d(oldEnding.getTranslation(), ending.getRotation());
    }

    // === Global Constants ===
    public static Rotation2d faceGoal = Rotation2d.fromDegrees(135);
    public static Rotation2d faceOpposingHP = Rotation2d.fromDegrees(90);
    public static Rotation2d faceGate = Rotation2d.fromDegrees(125);
    public static Rotation2d faceOrthogonalToLaunchZone = Rotation2d.fromDegrees(45);

    public static double maxLinearVelocity = 3.27;
    public static double maxLinearAcceleration = 5.0;

    public static Trajectory.mpParams highSpeedMotionProfile = new Trajectory.mpParams(maxLinearVelocity, maxLinearAcceleration);
    public static Trajectory.EndingConfig defaultEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, defaultLinger);
    public static Trajectory.EndingConfig fastEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, 0);

    // === Gate Intake Bezier ===
    public static Trajectory.mpParams gateMotionProfile = new Trajectory.mpParams(3.0, 3.0);
    public static Trajectory.EndingConfig gateEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, 2.5);

    public static Pose2d gateEnd = new Pose2d(14.0, 66.0, faceGate);

    // === Goal Bezier ===
    public static Trajectory.EndingConfig goalEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, defaultLinger);
    static double goalvalue = 20.0;
    public static Pose2d GoalCloseEnd = new Pose2d(-goalvalue, -goalvalue, Rotation2d.fromDegrees(-45.0));
    public static Pose2d GoalFarEnd = new Pose2d(-goalvalue, goalvalue, Rotation2d.fromDegrees(45.0));

    // === Back Bezier ===
    public static Trajectory.EndingConfig backEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, defaultLinger);
    public static Pose2d BackFarEnd = new Pose2d(62.5, -12, Rotation2d.fromDegrees(-90.0));
    public static Pose2d BackCloseEnd = new Pose2d(62.5, 12, Rotation2d.fromDegrees(90.0));

    // === HP Bezier ===
    public static Trajectory.EndingConfig hpEndingConfig = new Trajectory.EndingConfig(minimumParametricEnding, defaultLinger);
    public static Translation2d hp1Control = new Translation2d(45.0, -35.0);
    public static Translation2d hp2Control = new Translation2d(70.0, -40.0);
    public static Pose2d hpRedEnd = new Pose2d(62.5, -61, Rotation2d.fromDegrees(-90.0));
}
