package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

//@Config
public class PIDcontroller {

    // Public Static for Dashboard Use
    public static PIDCoefficients GAIN = new PIDCoefficients(0,0,0);

    private double setPoint, error, lastError, totalError, command, iTermLimit, deltaT;
    private long currentTime, lastTime;
    private boolean useIterm = false;

    /** Limit bound of the output. */

    /** /////////////////////////// CONSTRUCTORS \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public PIDcontroller(double Kp, double Kd, double updateRate) {
        GAIN.p = Kp;
        GAIN.d = Kd;
        GAIN.i = 0;
//        kp = Kp;
//        kd = Kd;
//        ki = 0;
        deltaT = 1/updateRate;
        useIterm = false;
        resetPIDController();
    }

    /* Constructor Overload */
    public PIDcontroller(double Kp, double Ki, double Kd, boolean enableIterm) {
        GAIN.p = Kp;
        GAIN.d = Kd;
        GAIN.i = Ki;

        useIterm = enableIterm;
        resetPIDController();
    }
    /** /////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ **/

    /*************************************************************************/
    public void setSetPoint(double setpoint) {
        setPoint = setpoint;
        resetPIDController();
    }

    /*************************************************************************/
    public double getCommand(double feedbackSensor) {
        double pTerm, dTerm, iTerm = 0;
        long deltaT = 0;

        error = setPoint - feedbackSensor;
//        deltaT = time - lastTime;

        pTerm = GAIN.p * error;
        dTerm = GAIN.d * (error - lastError) / deltaT;

        /** Enable below if using the Integral term **/
        if(useIterm) {
            totalError += error;
            iTerm = totalError + GAIN.i * error;

            if (iTerm > iTermLimit) {   // iTermSumLimit
                iTerm = iTermLimit;
            } else if (iTerm < iTermLimit) {    // -iTermSumLimit
                iTerm = iTermLimit; // Change to lower allowabe
            }

            lastError = error;
//            lastTime = time;
        }

        /********************************************/

        command = pTerm + dTerm + iTerm;
        return command;
    }

    public void resetPIDController() {
        totalError = 0;
        lastError = 0;
//        setSetPoint(0);
    }

    public void setCoeffs(double p, double d) {
        GAIN.p = p;
        GAIN.d = d;
    }


    /** Getters & Setters **/
    public double getPGain() { return GAIN.p;}
    public double getDGain() { return GAIN.d;}

}
