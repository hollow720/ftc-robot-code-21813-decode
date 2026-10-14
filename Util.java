package org.firstinspires.ftc.teamcode.util;

public class Util {

    public static double INCHES_PER_METER = 39.3701;
    public static double METERS_PER_INCH = 0.0254;

    /**
     * Takes Abs of input, and then returns 0 if below lower
     * @param input
     * @param lower
     * @return
     */
    public static double lowerDeadband(double input, double lower)
    {
        if (Math.abs(input) < lower){
            return 0;
        }

        else{
            return input;
        }
    }

    public static double millimetersToMeters(double a){
        return a / 1000.0;
    }

    public static double metersToMillimeters(double a){
        return a * 1000.0;
    }

    public static double metersToInches(double a) { return a * 39.3701; }
    public static double inchesToMeters(double a) { return a / 39.3701; }

    /**
     * Cubes the input
     * Since joystick input domain is [-1, 1]
     * And for abs(input) < 1, input^(3) < input
     *
     * @param input
     * @return
     */
    public static double cubicJoystickCurve(double input){
        return Math.pow(input, 3);
    }

    /**
     * Squares the input
     * Since joystick input domain is [-1, 1]
     * And for abs(input) < 1, input^(2) < input
     *
     * @param input
     * @return
     */
    public static double quadraticJoystickCurve(double input){
        return Math.signum(input) * Math.pow(input, 2);
    }

    public static double clamp(double maxValue, double minValue, double input)
    {
        if (input > maxValue)
        {
            return maxValue;
        }
        else if (input < minValue)
        {
            return minValue;
        }
        return input;
    }
}
