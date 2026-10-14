package org.firstinspires.ftc.teamcode.control.pathing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class Bezier {
    private Vector2d P0;
    private Vector2d P1;
    private Vector2d P2;
    private Vector2d P3;

    /**
     * Accepts in Inches
     * @param P0
     * @param P1
     * @param P2
     * @param P3
     */
    public Bezier(Vector2d P0, Vector2d P1, Vector2d P2, Vector2d P3)
    {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
    }

    /**
     * Overload for P2P
     * @param startPoint
     */
    public Bezier(Pose2d startPoint, Pose2d endPoint)
    {
        double midX = (startPoint.getX() + endPoint.getX()) / 2.0;
        double midY = (startPoint.getY() + endPoint.getY()) / 2.0;

        this.P0 = new Vector2d(startPoint.getX(), startPoint.getY());
        this.P1 = new Vector2d(midX, midY);
        this.P2 = new Vector2d(midX, midY);
        this.P3 = new Vector2d(endPoint.getX(), endPoint.getY());
    }

    /**
     * Pose2d + Translation for each control point
     * @param startPoint
     */
    public Bezier(Pose2d startPoint, Translation2d controlStartPoint, Translation2d controlEndPoint, Pose2d endPoint)
    {
        this.P0 = new Vector2d(startPoint.getX(), startPoint.getY());
        this.P1 = new Vector2d(controlStartPoint.getX(), controlStartPoint.getY());
        this.P2 = new Vector2d(controlEndPoint.getX(), controlEndPoint.getY());
        this.P3 = new Vector2d(endPoint.getX(), endPoint.getY());
    }


    public Vector2d get(double t) {
        double u = 1 - t;
        return P0.times(u*u*u)
                .plus(P1.times(3*u*u*t))
                .plus(P2.times(3*u*t*t))
                .plus(P3.times(t*t*t));
    }

    public Vector2d deriv(double t) {
        double u = 1 - t;
        return P1.minus(P0).times(3*u*u)
                .plus(P2.minus(P1).times(6*u*t))
                .plus(P3.minus(P2).times(3*t*t));
    }

    public Vector2d secondDerivative(double t)
    {
        double u = 1-t;
        return P2.minus(P1.times(2)).plus(P0).times(6 * u)
                .plus(P3.minus(P2.times(2)).plus(P1).times(6 * t));
    }

    public Bezier mirrorAllianceSide()
    {
        return new Bezier(
                new Vector2d(P0.getX(), -P0.getY()),
                new Vector2d(P1.getX(), -P1.getY()),
                new Vector2d(P2.getX(), -P2.getY()),
                new Vector2d(P3.getX(), -P3.getY())
        );
    }

    public Bezier reverse()
    {
        return new Bezier(
                new Vector2d(P3.getX(), P3.getY()),
                new Vector2d(P2.getX(), P2.getY()),
                new Vector2d(P1.getX(), P1.getY()),
                new Vector2d(P0.getX(), P0.getY())
        );
    }
}
