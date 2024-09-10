package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DevilDrive {

    private DcMotor lf, lr, rf, rr;
    public double k_snail = 0.5;    // % of full power multiplier (range 0 to 1)

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DevilDrive(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {

        lf = leftFront;
        lr = leftRear;
        rf = rightFront;
        rr = rightRear;

        /** Assign Motor Directions **/
        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.REVERSE);

        /** Initialize Motor Power to 0 **/
        setMotorPower(0,0,0,0);
    }

    /*************************************************************************/
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

    /*************************************************************************/
    private void setMotorPower(double lfPower,double rfPower, double lrPower, double rrPower) {
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lr.setPower(lrPower);
        rr.setPower(rrPower);
    }

    /** Accessors **/
    public double getLFpos(){ return lf.getCurrentPosition(); }
    public double geLRpos() { return lr.getCurrentPosition(); }
    public double geRFpos() { return rf.getCurrentPosition(); }
    public double geRRpos() { return rr.getCurrentPosition(); }
    public double getLFpower() {return lf.getPower(); }
    public double getLRpower() {return lr.getPower(); }
    public double getRFpower() {return rf.getPower(); }
    public double getRRpower() {return rr.getPower(); }





//    public String driveStats() {
//
//        JsonObject driveParams = new JsonObject();
//        driveParams.addProperty("LF Power", lf.getPowerFloat());
//        driveParams.addProperty("LR Power", lr.getPowerFloat());
//        driveParams.addProperty("RF Power", rf.getPowerFloat());
//        driveParams.addProperty("RR Power", rr.getPowerFloat());
//        driveParams.addProperty("LF Position", lf.getCurrentPosition());
//        driveParams.addProperty("LR Position", lr.getCurrentPosition());
//        driveParams.addProperty("RF Position", rf.getCurrentPosition());
//        driveParams.addProperty("RR Position", rr.getCurrentPosition());
//
//        return driveParams.getAsString();
//    }


}
