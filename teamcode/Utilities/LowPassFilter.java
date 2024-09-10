package org.firstinspires.ftc.teamcode.Utilities;

public class LowPassFilter {

    private double cutoffFrequency, period;
    protected double alpha = 0.8;
    double tau = 0;

    protected double previousEstimate = 0;

    /**
     *
     * @param alphaValue Filter Gain. (0 < alpha < 1)
     */
    public LowPassFilter(double alphaValue) {
        alpha = alphaValue;
    }

    public LowPassFilter(double cutoffFreq, double timePeriod) {
        cutoffFrequency = cutoffFreq;
        period = timePeriod;
        calculateAlpha();
    }

    /*************************************************************************/
    private void calculateAlpha() {
        tau = 1.0 / (2 * Math.PI * cutoffFrequency);
        alpha = period / (period + tau);
    }

    /************************************************************************
     * Low Pass Filter estimate
     * @param  rawValue measurement sensor signal
     * @return filtered value
     */
    public double estimate(double rawValue) {
//        double filteredValue = alpha * rawValue + (1-alpha) * previousEstimate;
        double filteredValue = alpha * previousEstimate + (1-alpha) * rawValue;

        previousEstimate = filteredValue;
        return filteredValue;
    }

    /*************************************************************************/
    public void setCutoffFrequency(double filterCutoff) {
        cutoffFrequency = filterCutoff;
        calculateAlpha();
    }
}
