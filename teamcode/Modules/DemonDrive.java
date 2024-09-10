package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.Utilities.PoseData;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DemonDrive {

    private DcMotor lf, lr, rf, rr;

    public double k_snail = 0.5;    // % of full power multiplier (range 0 to 1)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.025; //2;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.025; //15;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.035; //1;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double RANGE_OFFSET   = 14.0;  //  this is how close the camera should get to the target (inches)
    final double YAW_OFFSET     = 0;    // This is for TFOD camera location
    final double HEADING_OFFSET = 0;     // Change if camera is relocated

    /** Class Constructor MecDrive **/
    public DemonDrive(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {

        lf = leftFront;
        lr = leftRear;
        rf = rightFront;
        rr = rightRear;

        /** Assign Motor Directions **/
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.FORWARD);

        /** Initialize Motor Power to 0 **/
        setMotorPower(0,0,0,0);
    }

    /**
     * This function calculates motor power for manual control of mecanum drive
     * @param Drive     joystick input
     * @param Strafe    Joystick Input
     * @param Turn      Joystick Input
     */
    public void mecanumDrive(double Drive, double Strafe, double Turn) {

        double y  = Drive;
        double x  = Strafe;
        double rx = Turn;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower  = (y+x+rx) / denom;
        double backLeftPower   = (y-x+rx) / denom;
        double frontRightPower = (y-x-rx) / denom;
        double backRightPower  = (y+x-rx) / denom;

//        double denominator = Math.max(Math.abs(Drive) + Math.abs(Strafe) + Math.abs(Turn), 1);
//        double frontLeftPower  = (Drive + Strafe + Turn) / denominator;
//        double backLeftPower   = (Drive - Strafe + Turn) / denominator;
//        double frontRightPower = (Drive - Strafe - Turn) / denominator;
//        double backRightPower  = (Drive + Strafe - Turn) / denominator;
        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setMotorPower(double lfPower,double rfPower, double lrPower, double rrPower) {
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lr.setPower(lrPower);
        rr.setPower(rrPower);
    }

    /** Accessors **/
    public double getLFpos()   { return lf.getCurrentPosition(); }
    public double geLRpos()    { return lr.getCurrentPosition(); }
    public double geRFpos()    { return rf.getCurrentPosition(); }
    public double geRRpos()    { return rr.getCurrentPosition(); }
    public double getLFpower() { return lf.getPower(); }
    public double getLRpower() { return lr.getPower(); }
    public double getRFpower() { return rf.getPower(); }
    public double getRRpower() { return rr.getPower(); }





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
