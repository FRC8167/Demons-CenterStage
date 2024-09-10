package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MOD_Drive {

    private DcMotor lf, lr, rf, rr;
    public double k_snail = 0.5;    // % of full power multiplier (range 0 to 1)

    /** Class Constructor MecDrive **/
    public MOD_Drive(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {

        lf = leftFront;
        lr = leftRear;
        rf = rightFront;
        rr = rightRear;

        /** Assign Motor Directions **/


//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightRear.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.FORWARD);

        /** Initialize Motor Power to 0 **/
        setMotorPower(0,0,0,0);
    }

    public void mecanumDrive(double Drive, double Strafe, double Turn) {

        double y = Drive;
        double x = Strafe;
        double rx = Turn;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y+x+rx) / denom;
        double backLeftPower = (y-x+rx) / denom;
        double frontRightPower = (y-x-rx) / denom;
        double backRightMotor = (y+x-rx) / denom;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightMotor);
    }

    private void setMotorPower(double lfPower,double rfPower, double lrPower, double rrPower) {
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lr.setPower(lrPower);
        rr.setPower(rrPower);
    }


}
