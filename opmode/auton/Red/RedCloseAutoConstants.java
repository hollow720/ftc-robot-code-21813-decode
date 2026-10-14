package org.firstinspires.ftc.teamcode.opmode.auton.Red;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.control.pathing.Bezier;
import org.firstinspires.ftc.teamcode.control.pathing.Trajectory;

public class RedCloseAutoConstants extends RedGlobalAutoConstants {
    public static Pose2d faceHPScoringPosition = new Pose2d(-15.0, 10.0, faceOpposingHP);
    public static Pose2d cycleScoringPosition = new Pose2d(faceHPScoringPosition.getTranslation(), Rotation2d.fromDegrees(135));

    // === Goal To Scoring ===
    public static Pose2d P1S = new Pose2d(-48.0, 48.0, faceGoal);
    public static Pose2d P1E = faceHPScoringPosition;
    public static Bezier bezier1 = new Bezier(P1S, P1E);
    public static Trajectory trajectory1 = new Trajectory(bezier1, highSpeedMotionProfile, defaultEndingConfig, P1S, P1E);

    // === Scoring to Middle ===
    public static Pose2d P2S = P1E;
    public static Translation2d P2C1 = new Translation2d(16.0, 18.0);
    public static Translation2d P2C2 = P2C1;
    public static Pose2d P2E = new Pose2d(16.0, 70.0, faceOpposingHP);
    public static Bezier bezier2 = new Bezier(
            P2S,
            P2C1,
            P2C2,
            P2E);
    public static Trajectory trajectory2 = new Trajectory(bezier2, gateMotionProfile, fastEndingConfig, P2S, P2E);

    // === Middle To Scoring ===
    public static Trajectory trajectory3 = trajectory2.reversePath(cycleScoringPosition.getRotation(), defaultEndingConfig);

    // === Middle To Gate ===
    public static Pose2d P4S = trajectory3.getEndingPose();
    public static Translation2d P4C1 = P2C1;
    public static Translation2d P4C2 = P2C1;
    public static Pose2d P4E =
            new Pose2d(gateEnd.getTranslation().plus(new Translation2d(0, -10)), gateEnd.getRotation());
    public static Bezier bezier4 = new Bezier(
            P4S,
            P4C1,
            P4C2,
            P4E);
    public static Trajectory trajectory4 = new Trajectory(bezier4, highSpeedMotionProfile, fastEndingConfig, P4S, P4E);

    // === Gate To Scoring ===
    public static Trajectory trajectory5A = trajectory4.reversePath(highSpeedMotionProfile, defaultEndingConfig);

    // === Gate To Scoring Final
//    public static Trajectory trajectory5B = trajectory4.reversePath(faceOpposingHP, highSpeedMotionProfile, defaultEndingConfig);
    public static Trajectory trajectory5B = trajectory5A;


    // === Close 3 ===
    public static Pose2d P6S = new Pose2d(trajectory5A.getEndingPose().getTranslation(), faceHPScoringPosition.getRotation());
    public static Pose2d P6E = new Pose2d(faceHPScoringPosition.getX(), 58.0, faceOpposingHP);
    public static Bezier bezier6 = new Bezier(P6S, P6E);
    public static Trajectory trajectory6 = new Trajectory(bezier6, highSpeedMotionProfile, fastEndingConfig, P6S, P6E);

    // === Close 3 Back Leave Position ===

    public static Pose2d P7E = new Pose2d(-55, 14, faceOrthogonalToLaunchZone);
    public static Pose2d P7S = trajectory6.getEndingPose();
    public static Bezier bezier7 = new Bezier(P7S, P7E);
    public static Trajectory trajectory7A = new Trajectory(bezier7, highSpeedMotionProfile, defaultEndingConfig, P7S, P7E);

    // === Close 3 Reverse

    public static Pose2d P7SB = P6E;
    public static Pose2d P7EB = faceHPScoringPosition;
    public static Bezier bezier7B = new Bezier(P7SB, P7EB);
    public static Trajectory trajectory7B = new Trajectory(bezier7B, highSpeedMotionProfile, defaultEndingConfig, P7SB, P7EB);

    // === Far 3 ===
    public static Pose2d P8S = trajectory7B.getEndingPose();
    public static Translation2d P8C1 = new Translation2d(46.0, 16.0);
    public static Translation2d P8C2 = P8C1;
    public static Pose2d P8E = new Pose2d(44.0, 70.0, faceOpposingHP);
    public static Bezier bezier8 = new Bezier(
            P8S,
            P8C1,
            P8C2,
            P8E);
    public static Trajectory trajectory8 = new Trajectory(bezier8, highSpeedMotionProfile, defaultEndingConfig, P8S, P8E);

    // === Far 3 leave pathing ===
    public static Pose2d P9E = P7E;
    public static Translation2d P9C1 = new Translation2d(44.0, 20.0);
    public static Translation2d P9C2 = P9C1;
    public static Pose2d P9S = P8E;
    public static Bezier bezier9 = new Bezier(P9S, P9C1, P9C2, P9E);
    public static Trajectory trajectory9 = new Trajectory(bezier9, highSpeedMotionProfile, defaultEndingConfig, P9S, P9E);

}
