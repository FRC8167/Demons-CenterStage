package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DemonPivot {

    DcMotorEx pivotMotor;

    /** Preset Levels **/
    public final int POS_NEST = 20;
    public final int POS_FLOOR_PICKUP = 200;
    public final int POS_SCORE_LOW = 539;
    public final int POS_SCORE_HIGH = 1500;
    public final int POS_TRAVEL = 800;

    /** Travel Limits and Control Params **/
    public final int POS_LIMIT_LOW = 0;
    public final int POS_LIMIT_HIGH = 2100;
    private final double CTRL_POWER = 0.85;
    private final double CTRL_RATE = 300;       // Default rate of 300 ticks/sec

    private double kp = 8;
    private final int POS_TOLERANCE = 25;


    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DemonPivot(DcMotorEx pivotMotr) {
        pivotMotor = pivotMotr;
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setPidParms(kp);
    }


    /*************************************************************************/
    public void manualPivotRotate(double setPoint) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setPower(clampPowerSetpt(setPoint));
    }

    /*************************************************************************/
    public void ffRotate(double setPoint) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setPower(clampPowerSetpt(setPoint));
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

//            pivotMotor.setPositionPIDFCoefficients(2);
//            pivotMotor.setTargetPositionTolerance(40);

        pivotMotor.setTargetPosition(clampPositionSetpt(trgtPosCnts));
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setVelocity(CTRL_RATE);

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

        int currentPosition = pivotMotor.getCurrentPosition();
        double rate = (trgtPosCnts - currentPosition) / duration_sec;

        pivotMotor.setTargetPosition(clampPositionSetpt(trgtPosCnts));
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setVelocity(rate);

    }


    /*************************************************************************/
    public boolean motionFinished() {
        int error = Math.abs(pivotMotor.getCurrentPosition() - pivotMotor.getTargetPosition());
        if (error < POS_TOLERANCE) return true; else return false;

        // Also try using the isBusy() method
    }


    /*************************************************************************/
    public void update() {
        int currentPos = pivotMotor.getCurrentPosition();

        if (currentPos > POS_LIMIT_HIGH || currentPos < POS_LIMIT_LOW) {
            pivotMotor.setPower(0);
        }
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
        if (setPoint > POS_LIMIT_HIGH) {
            setPoint = POS_LIMIT_HIGH;
        } else if (setPoint < POS_LIMIT_LOW) {
            setPoint = POS_LIMIT_LOW    ;
        }
        return setPoint;
    }

    /*************************************************************************/
    public void resetEncoder() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*************************************************************************/
    public void setFeedForwardTerm(double feedForwardGain){
        PIDFCoefficients gains = new PIDFCoefficients(kp, 0, 0, feedForwardGain);
        pivotMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
    }

    /*************************************************************************/
    public void setPidParms(double pGain) {
        kp = pGain;
        pivotMotor.setPositionPIDFCoefficients(kp);
    }


    /** Accessors **/
    public int getPivotPos() {return pivotMotor.getCurrentPosition(); }
    public double getPivotPower() {return pivotMotor.getPower(); }
    public double getPivotCurrent() {return pivotMotor.getCurrent(CurrentUnit.MILLIAMPS); }

}
