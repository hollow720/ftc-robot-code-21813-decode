package org.firstinspires.ftc.teamcode.control.pathing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.PathingConstants;
import org.firstinspires.ftc.teamcode.control.kinematics.ChassisSpeed;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.WPIPID;

public class mpFollower extends Follower {
    Trajectory trajectory;
    ElapsedTime sharedTimer;

    private WPIPID posTrackingController;
    private WPIPID vxTrackingController;
    private WPIPID vyTrackingController;
    private WPIPID wController;
    PinpointSubsystem pinpointSubsystem;

    boolean timeOverload = false;

    public mpFollower()
    {
        posTrackingController = new WPIPID(PathingConstants.kP_x, 0, 0);
        vxTrackingController = new WPIPID(PathingConstants.kP_v, 0, 0);
        vyTrackingController = new WPIPID(PathingConstants.kP_v, 0, 0);
        wController =    new WPIPID(PathingConstants.kP_w, 0, 0);

        wController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void start(Trajectory trajectory, ElapsedTime sharedTimer, PinpointSubsystem pinpointSubsystem)
    {
        this.trajectory = trajectory;
        this.sharedTimer = sharedTimer;
        this.pinpointSubsystem = pinpointSubsystem;

        this.timeOverload = false;

        this.sharedTimer.reset();

        posTrackingController.reset();
        vxTrackingController.reset();
        vyTrackingController.reset();
        wController.reset();
    }

    public void timeOverload()
    {
        this.timeOverload = true;
    }

    /**
     * Returns field relative chassis speed
     * @return
     */
    public ChassisSpeed calculate()
    {
        Trajectory.TrajectoryState state = trajectory.sample(sharedTimer.seconds());
        Pose2d target = state.targetPose; //Ignore the rotation aspect and just use interpolation

        if (sharedTimer.seconds() > trajectory.getPathDuration() + 0.25)
        {
            vxTrackingController.setP(0);
            vyTrackingController.setP(0);

            posTrackingController.setP(3 * posTrackingController.getP());
        }

        //Position PID feedbcak
        Vector2d positionErrorVector = new Vector2d(MathUtil.applyDeadband(target.getX() - pinpointSubsystem.getPose().getX(), 1e-4), MathUtil.applyDeadband(target.getY()-pinpointSubsystem.getPose().getY(), 1e-4));
        double error = positionErrorVector.magnitude();

        Vector2d correctionVector = positionErrorVector.normalize()
                .times(PathingConstants.kP_x * error);

        //So like measurement and error are swapped on purpose so thati don't negate my correction vector in the wrong direction.

        //Heading PID
        double rotationEffort = wController.calculate(pinpointSubsystem.getPose().getHeading(), target.getHeading());

        // So in theory this should just work but because i dont run velocity PID because laZ
        // So i actually have to run the PID on velocity here or else it wont decelerate properly

        double pvx = vxTrackingController.calculate(pinpointSubsystem.getFieldRelativeVelocityMetric().getX(), state.fieldVelocity.getX());
        double pvy = vyTrackingController.calculate(pinpointSubsystem.getFieldRelativeVelocityMetric().getY(), state.fieldVelocity.getY());

        double vx = pvx + correctionVector.getX();
        double vy = pvy + correctionVector.getY();

        ChassisSpeed cs = new ChassisSpeed(vx, vy, rotationEffort);

//        Log.i("pathing - current pinpoint pos", pinpointSubsystem.getPose().toString());
//        Log.i("pathing - target pos", target.toString());
//        Log.i("pathing - chassis speed x", String.format("%s", cs.getVx()));
//        Log.i("pathing - chassis speed y", String.format("%s", cs.getVy()));
//        Log.i("pathing - chassis speed w", String.format("%s", cs.getOmega()));
//        Log.i("pathing - errorVector", errorVector.toString());
//        Log.i("pathing - correctionVector", correctionVector.toString());
//        Log.i("pathing - rotationEffort", String.format("%s", rotationEffort));

        return cs;
    }

    public boolean isFinished()
    {
        if (timeOverload)
        {
            return pastTimeCondition();
        }
        return withinParametricTolerance() || pastTimeCondition();
    }

    public boolean withinParametricTolerance() // this is time based also lol
    {
        return trajectory.withinParametricTolerance(sharedTimer.seconds());
    }

    public boolean pastTimeCondition()
    {
        return sharedTimer.seconds() > (trajectory.getPathDuration() + trajectory.getPathLinger());
    }
}
