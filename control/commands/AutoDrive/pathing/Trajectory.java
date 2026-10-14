package org.firstinspires.ftc.teamcode.control.pathing;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.TreeMap;

public class Trajectory extends LogAccess {
    private final Bezier bezier;
    private final EndingConfig endingConfig;
    private final mpParams mpParameters;
    private final TimedMotionProfile motionProfile;
    private TreeMap<Double, Double> arcLengthInchesToParameter = new TreeMap<>();
    private double parameterIncrement = 0.01;
    private double arcLengthInches;
    public Rotation2d startRot;
    public Rotation2d endRot;
    public Trajectory(Bezier bezier, mpParams mpParameters, EndingConfig endingConfig, Rotation2d startRot, Rotation2d endRot)
    {
        this.bezier = bezier;
        this.endingConfig = endingConfig;
        this.mpParameters = mpParameters;
        this.startRot = startRot;
        this.endRot = endRot;

        buildArcParameterization();
        motionProfile = new TimedMotionProfile(mpParameters, Util.inchesToMeters(arcLengthInches));
        for (double i = 0; i < motionProfile.duration(); i += 0.01)
        {
            Log.i("pathing - t parametric at time " + i, String.format("%s", getParametric_t(i)));
        }
    }

    public Trajectory(Bezier bezier, mpParams mpParameters, EndingConfig endingConfig, Pose2d startRot, Pose2d endRot)
    {
        this.bezier = bezier;
        this.endingConfig = endingConfig;
        this.mpParameters = mpParameters;
        this.startRot = startRot.getRotation();
        this.endRot = endRot.getRotation();

        buildArcParameterization();
        motionProfile = new TimedMotionProfile(mpParameters, Util.inchesToMeters(arcLengthInches));
        for (double i = 0; i < motionProfile.duration(); i += 0.01)
        {
            Log.i("pathing - t parametric at time " + i, String.format("%s", getParametric_t(i)));
        }
    }

    public Trajectory mirrorAllianceSide()
    {
        return new Trajectory(
                this.bezier.mirrorAllianceSide(),
                this.mpParameters,
                this.endingConfig,
                this.startRot.times(-1.0),
                this.endRot.times(-1.0)
        );
    }

    public Trajectory reversePath()
    {
        return new Trajectory(
                this.bezier.reverse(),
                this.mpParameters,
                this.endingConfig,
                this.endRot,
                this.startRot
        );
    }

    public Trajectory reversePath(Rotation2d newEndRot)
    {
        return new Trajectory(
                this.bezier.reverse(),
                this.mpParameters,
                this.endingConfig,
                this.endRot,
                newEndRot
        );
    }

    public Trajectory reversePath(Rotation2d newEndRot, EndingConfig endingConfig)
    {
        return new Trajectory(
                this.bezier.reverse(),
                this.mpParameters,
                endingConfig,
                this.endRot,
                newEndRot
        );
    }

    public Trajectory reversePath(Rotation2d newEndRot, mpParams motionProfile, EndingConfig endingConfig)
    {
        return new Trajectory(
                this.bezier.reverse(),
                this.mpParameters,
                endingConfig,
                this.endRot,
                newEndRot
        );
    }

    public Trajectory reversePath(Rotation2d newEndRot, mpParams motionProfile)
    {
        return new Trajectory(
                this.bezier.reverse(),
                motionProfile,
                this.endingConfig,
                this.endRot,
                newEndRot
        );
    }

    public Trajectory reversePath(mpParams mpParameters)
    {
        return this.reversePath(
                mpParameters, this.endingConfig
        );
    }

    public Trajectory reversePath(mpParams mpParameters, EndingConfig endingConfig)
    {
        return new Trajectory(
                this.bezier.reverse(),
                mpParameters,
                endingConfig,
                this.endRot,
                this.startRot
        );
    }

    public Pose2d getStartingPose()
    {
        return new Pose2d(
                this.bezier.get(0).getX(),
                this.bezier.get(0).getY(),
                this.startRot
        );
    }

    public Pose2d getEndingPose()
    {
        return new Pose2d(
                this.bezier.get(1).getX(),
                this.bezier.get(1).getY(),
                this.endRot
        );
    }

    public mpParams getParams()
    {
        return this.mpParameters;
    }

    private void buildArcParameterization()
    {
        double lastLength = 0;
        Vector2d lastPoint = bezier.get(0);

        arcLengthInchesToParameter.put(0.0, 0.0);

        for (double i = parameterIncrement; i <= 1; i+= parameterIncrement)
        {
            double newLength = lastLength + bezier.get(i).minus(lastPoint).magnitude();
            arcLengthInchesToParameter.put(newLength, i);

            lastLength = newLength;
            lastPoint = bezier.get(i);
            Log.v("pathing - total bezier length", String.format("%s", lastLength));
        }

        if (!arcLengthInchesToParameter.containsValue(1.0))
        {
            arcLengthInchesToParameter.put(lastLength, 1.0);
            Log.v("pathing - total bezier length", String.format("%s", lastLength));
        }

        try
        {
            arcLengthInches = arcLengthInchesToParameter.lastKey();
        }
        catch (NullPointerException e)
        {
            Log.e("Trajectory Failed To Generate", "arcLength is null");
            throw e;
        }
    }

    public Pose2d getPosAtArcLength(double sInches)
    {
        double t;
        try
        {
            t = arcLengthInchesToParameter.floorEntry(sInches).getValue();
        }
        catch (NullPointerException e)
        {
            Log.e("Parameter t is null in TreeMap", "Parameter t is null");
            throw e;
        }
        Vector2d position = bezier.get(t);
        Vector2d dposition = bezier.deriv(t); //doesnt matter if its inches or not

        Rotation2d heading = new Rotation2d(dposition.getX(), dposition.getY());

        return new Pose2d(
                position.getX(),
                position.getY(),
                heading
        );
    }

    public TrajectoryState sample(double t_time)
    {
        t_time = MathUtil.clamp(t_time, 0, motionProfile.duration());
        Log.i("Pathing - t_time", String.format("%s", t_time));
        double t_geom = 0;
        try {
            t_geom = arcLengthInchesToParameter.floorEntry(Util.metersToInches(motionProfile.getPosition(t_time))).getValue();
        }
        catch (NullPointerException e)
        {
            t_geom = 0;
            Log.e("map retrieval failed", "mapping from distance to parameter");
        }

        double eps = 1e-12;

        // Geometry computations are still in inches
        Vector2d positionInches = bezier.get(t_geom);
        Vector2d dpositionInches = bezier.deriv(t_geom);

        double linearVelocity = motionProfile.getVelocity(t_time);
        Vector2d normalized = dpositionInches.normalize();
        Vector2d velocity = normalized.times(linearVelocity);

        //Heading interpolation
        Rotation2d targetRotation = interpolateRotParametric(this.startRot, this.endRot, t_geom);

        return new TrajectoryState(
                new Pose2d(positionInches.getX(), positionInches.getY(), targetRotation),
                velocity
        );
    }

    public Rotation2d interpolateRotParametric(Rotation2d r1, Rotation2d r2, double t)
    {
        // Normalize t between 0 and 1
        t = Math.max(0.0, Math.min(1.0, t));

        // Difference between angles, wrapped to (-π, π]
        double delta = r2.minus(r1).getRadians();

        // Interpolated angle = start + fraction of delta
        double interpolated = r1.getRadians() + delta * t;

        return new Rotation2d(interpolated);
    }

    public boolean withinParametricTolerance(double t_time)
    {
        return this.getParametric_t(t_time) >= endingConfig.minimumParametricEnding;
    }

    private double getParametric_t(double t_time)
    {
        t_time = MathUtil.clamp(t_time, 0, motionProfile.duration());
        double t_geom = 0;
        try {
            t_geom = arcLengthInchesToParameter.floorEntry(Util.metersToInches(motionProfile.getPosition(t_time))).getValue();
        }
        catch (NullPointerException e)
        {
            t_geom = 0;
            Log.e("map retrieval failed", "mapping from distance to parameter");
        }

        return t_geom;
    }

    public double getPathDuration()
    {
        return this.motionProfile.duration();
    }

    public double getPathLinger()
    {
        return this.endingConfig.pathLinger;
    }

    private TimedMotionProfile getMotionProfile()
    {
        return this.motionProfile;
    }

    public Bezier getBezier()
    {
        return this.bezier;
    }

    public static class mpParams
    {
        public final double maxLinearVelocity;
        public final double maxLinearAcceleration;

        public mpParams(double maxLinearVelocity, double maxLinearAcceleration)
        {
            this.maxLinearVelocity = maxLinearVelocity;
            this.maxLinearAcceleration = maxLinearAcceleration;
        }
    }

    public static class TrajectoryState
    {
        public final Pose2d targetPose;
        public final Vector2d fieldVelocity;
        public TrajectoryState(Pose2d targetPose, Vector2d fieldVelocity)
        {
            this.targetPose = targetPose;
            this.fieldVelocity = fieldVelocity;
        }
    }

    public static class TrajectoryStateLQR
    {
        public final Pose2d targetPose;
        public final double forwardVelocity;
        public final double angularVelocity;
        public final double forwardAcceleration;
        public final double angularAcceleration;

        public TrajectoryStateLQR(Pose2d targetPose, double forwardVelocity, double angularVelocity, double forwardAcceleration, double angularAcceleration)
        {
            this.targetPose = targetPose;
            this.forwardVelocity = forwardVelocity;
            this.angularVelocity = angularVelocity;
            this.forwardAcceleration = forwardAcceleration;
            this.angularAcceleration = angularAcceleration;
        }
    }

    public static class TimedMotionProfile
    {
        mpParams mpParameters;
        double distance;
        double accelTime;
        double decelTime;
        double cruiseTime;
        double maxLinearVelocity;
        public TimedMotionProfile(mpParams mpParameters, double distance)
        {
            this.mpParameters = mpParameters;
            this.distance = distance;

            generate();
        }

        public void generate()
        {
            accelTime = Math.min(Math.pow(distance/ mpParameters.maxLinearAcceleration, 0.5), mpParameters.maxLinearVelocity/ mpParameters.maxLinearAcceleration);
            decelTime = accelTime;

            this.maxLinearVelocity = Math.min(mpParameters.maxLinearVelocity, mpParameters.maxLinearAcceleration * accelTime);

            double remainingDistance = Math.max(0, distance - mpParameters.maxLinearAcceleration * Math.pow(accelTime, 2));
            cruiseTime = remainingDistance/this.maxLinearVelocity;
            Log.i("pathing - mp accel time", String.format("%s", accelTime));
            Log.i("pathing - mp cruise time", String.format("%s", cruiseTime));
            Log.i("pathing - mp decel time", String.format("%s", decelTime));
            Log.i("pathing - duration()", String.format("%s", duration()));
        }

        //For s(t) v(t) and a(t) model it piecewise in the 3 phases

        public double getPosition(double time)
        {
            double position;
            if (time < accelTime) //First phase of trapezoidal profile
            {
                position = 0 + 0 + 0.5 * mpParameters.maxLinearAcceleration * Math.pow(time, 2);
            }
            else if (time < cruiseTime +accelTime) //Constant velocity phase of trapezoidal profile
            {
                position = 0.5 * mpParameters.maxLinearAcceleration * Math.pow(accelTime, 2) + this.maxLinearVelocity * (time - accelTime);
            }
            else //Decelerating phase of trapezoidal profile
            {
                position = 0.5 * mpParameters.maxLinearAcceleration * Math.pow(accelTime, 2) + this.maxLinearVelocity * (cruiseTime) + this.maxLinearVelocity * (time-accelTime- cruiseTime) + (0.5) * (-mpParameters.maxLinearAcceleration) * Math.pow(time-accelTime- cruiseTime, 2);
            }

            return position;
        }

        public double getVelocity(double time)
        {
            double velocity = 0;
            if (time < accelTime)
            {
                velocity = mpParameters.maxLinearAcceleration * time;
            }
            else if (time < (accelTime + cruiseTime))
            {
                velocity = this.maxLinearVelocity;
            }
            else if (time < (accelTime + cruiseTime + decelTime))
            {
                velocity = this.maxLinearVelocity - (time-accelTime- cruiseTime) * mpParameters.maxLinearAcceleration;
            }

            return velocity;
        }

        public double getAcceleration(double time)
        {
            double acceleration;
            if (time < accelTime)
            {
                acceleration = mpParameters.maxLinearAcceleration;
            }
            else if (time < accelTime + cruiseTime)
            {
                acceleration = 0;
            }
            else {
                acceleration = -mpParameters.maxLinearAcceleration;
            }

            return acceleration;
        }

        public double duration()
        {
            return 2 * this.accelTime + this.cruiseTime;
        }
    }

    public static class EndingConfig
    {
        public double minimumParametricEnding;
        public double pathLinger;

        public EndingConfig(double minimumParametricEnding, double pathLinger)
        {
            this.minimumParametricEnding = minimumParametricEnding;
            this.pathLinger = pathLinger;
        }
    }
}
