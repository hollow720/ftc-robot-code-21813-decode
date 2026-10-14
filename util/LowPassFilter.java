package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

public class LowPassFilter {
    private double state = 0.0;
    private double t = 0.05;
    private double alpha = (0.02)/(t + 0.02);

    // for a standard exponential low pass
    // a = dt/(t + dt)
    // dt - loop period (s)
    // t - filter time constant

    /**
     * AI LOL
     * What the time constant τ actually is
     *
     * τ is how fast the filter reacts.
     *
     * Formally (first-order low-pass):
     *
     * After τ seconds, the output reaches 63% of a sudden change
     *
     * After 2τ → ~86%
     *
     * After 3τ → ~95%
     *
     * So τ directly = latency vs smoothness tradeoff.
     */


    public LowPassFilter()
    {

    }

    public double calculate(double val)
    {
        // y_k = y_k-1 + a(x-y_k-1);
        state = state + alpha * (val-state);

        return sample();
    }

    public double sample() {
        return state;
    }
}
