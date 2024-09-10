package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DemonSlide {

    DcMotorEx slideMotor;

    /** Preset Levels **/
    public final int POS_NEST = 20;
    public final int POS_FLOOR_PICKUP = 300;
    public final int POS_SCORE_LOW = 493;
    public final int POS_SCORE_HIGH = 1600;
    public final int POS_TRAVEL = 40;

    /** Travel Limits and Control Params **/
    public final int RETRACT_LIMIT = -10;
    public final int EXTEND_LIMIT = 1675;

    private final double CTRL_POWER = 0.6;
    private final double CTRL_RATE = 300;  /** counts/sec **/

    private double kp = 6;
    private double kf = 0;
    private final int POS_TOLERANCE = 25;

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DemonSlide(DcMotorEx slideMtr) {
        slideMotor = slideMtr;
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setPidParms(kp);

        /** Zero Encoder - This may not be the right place **/
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*************************************************************************/
    public double clampPowerSetpt(double setPoint) {

        if (setPoint > 1.0) {
            setPoint = 1.0;
        } else if (setPoint < -1.0) {
            setPoint = -1.0;
        }
        return setPoint;
    }

    /*************************************************************************/
    public int clampPositionSetpt(int setPoint) {
        if (setPoint > EXTEND_LIMIT) {
            setPoint = EXTEND_LIMIT;
        } else if (setPoint < RETRACT_LIMIT) {
            setPoint = RETRACT_LIMIT;
        }
        return setPoint;
    }

    /*************************************************************************/
    public void manualSlideMove(double setPoint) {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(clampPowerSetpt(setPoint));
    }

    /*************************************************************************/
    public void moveToCountPosition(int trgtPosCnts, double duration_sec) {

        /** https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
         *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
         *
         *  DcMotorEx method RUN_TO_POSITION must be done in the following order:
         *      1. Set target position [encoder counts]
         *      2. Set motor mode to RUN_TO_POSITION
         *      3. Set the maximum velocity you want the motor to move
         *          motor.setVelocity(rate) [counts/sec]
         *          motor.setVelocity(rate, units) [ AngleUnit.DEGREES/sec or AngleUnit.RADIANS/sec]
         */

//            pivotMotor.setPositionPIDFCoefficients(2);
//            pivotMotor.setTargetPositionTolerance(40);

        int currentPosition = slideMotor.getCurrentPosition();
        double rate = (trgtPosCnts - currentPosition) / duration_sec;

        slideMotor.setTargetPosition(clampPositionSetpt(trgtPosCnts));
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(rate);

    }

    /*************************************************************************/
    public void moveToPosition(int trgtPosCnts) {

        /** https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
         *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
         *
         *  DcMotorEx method RUN_TO_POSITION must be done in the following order:
         *      1. Set target position [encoder counts]
         *      2. Set motor mode to RUN_TO_POSITION
         *      3. Set the maximum velocity you want the motor to move
         *          motor.setVelocity(rate) [counts/sec]
         *          motor.setVelocity(rate, units) [ AngleUnit.DEGREES/sec or AngleUnit.RADIANS/sec]
         */

        slideMotor.setTargetPosition(clampPositionSetpt(trgtPosCnts));
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(CTRL_RATE);

    }

    /*************************************************************************/
    public boolean motionFinished() {
//        int error = Math.abs(slideMotor.getCurrentPosition() - setPointValue);
        int error = Math.abs(slideMotor.getCurrentPosition() - slideMotor.getTargetPosition());
        if (error < POS_TOLERANCE) return true; else return false;
    }

    /*************************************************************************/
    public void update() {
        int currentPos = slideMotor.getCurrentPosition();

        if (currentPos > EXTEND_LIMIT || currentPos < RETRACT_LIMIT) {
            slideMotor.setPower(0);
        }
    }

    /*************************************************************************/
    public void setPidParms(double pGain) {
        kp = pGain;
        slideMotor.setPositionPIDFCoefficients(kp);
    }

    /*************************************************************************/
    public void setFeedForward(double FFgain) {
        kf = FFgain;
        slideMotor.setPositionPIDFCoefficients(kp);

    }

    /*************************************************************************/
    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** Accessors **/
    public int getSlidePos() {return slideMotor.getCurrentPosition(); }
    public double getSlidePower() {return slideMotor.getPower(); }
}
