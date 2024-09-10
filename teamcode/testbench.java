package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class testbench extends LinearOpMode {
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor zero;
    private DcMotor one;
    private DcMotor two;
    private DcMotor three;
    //private double starPosition, bladePosition;

    //private MecanumDrive drive;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //public static final double SERVO_CENTER = 0.5, SERVO_MIN = 0.0, SERVO_MAX = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        zero = hardwareMap.get(DcMotor.class, "zero");
        one = hardwareMap.get(DcMotor.class, "one");
        two = hardwareMap.get(DcMotor.class, "two");
        three = hardwareMap.get(DcMotor.class, "three");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        // ----------- ENCODER -------------
        // reset encoder counts kept by motors.
//        turtle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run to target encoder position and stop with brakes on
        telemetry.addData("Mode", "waiting");
        telemetry.update();

//        turtle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rabbit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // set motors to run forward
//        turtle.setTargetPosition(538);  //538
//        turtle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        starPosition = SERVO_CENTER;
//        bladePosition = SERVO_CENTER;
//        star.setPosition(starPosition);
//        blade.setPosition(bladePosition);


        waitForStart();

//        telemetry.addData("Mode", "running");
//        telemetry.addData("encoder", turtle.getCurrentPosition() + "something");
//        telemetry.update();
        zero.setPower(0.0);
        one.setPower(0.0);
        two.setPower(0.0);
        three.setPower(0.0);

//        while (turtle.isBusy()){
//            telemetry.addData("Motor", "running!!!!!");
//           telemetry.update();
//        }
//        turtle.setPower(0);
////        turtle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turtle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
//        star.setPosition(SERVO_CENTER);
//        blade.setPosition(SERVO_CENTER);
//        turtle.setPower(0);
//        rabbit.setPower(0);


        while (opModeIsActive()) {

            double allPower = gamepad1.left_stick_y;

            zero.setPower(allPower);
            one.setPower(allPower);
            two.setPower(allPower);
            three.setPower(allPower);
        }
    }

}

//            double leftY = -gamepad1.left_stick_y;
//            double rightY = -gamepad1.right_stick_y;
//            double turtlePower = leftY;
//            double rabbitPower = rightY;
//
//            turtle.setPower(turtlePower);
//            rabbit.setPower(rabbitPower);
//
//            if (gamepad1.a) {
//                starPosition = SERVO_CENTER;
//                bladePosition = SERVO_CENTER;
//            }
//
//            if (gamepad1.b) {
//                if (starPosition < 1) {
//                    starPosition += 0.001;
//                }
//            }
//
//            if (gamepad1.y) starPosition = SERVO_CENTER;
//
//            if (gamepad1.x) {
//                if (starPosition > 0) {
//                    starPosition -= 0.001;
//                }
//            }
//
//            star.setPosition(starPosition);
//            blade.setPosition(starPosition);

            //-------pixel pick-up test-------

//            boolean clawClosed = False;

//            if (gamepad1.y) {
//                if (bladePosition > 0) { //if position is 90 degrees (open claw)
//                    bladePosition = 0.7;
//                }
//                if (starPosition == SERVO_CENTER) {
//                    starPosition = 0.3;

//                clawClosed = True;


//            if (gamepad1.right_bumper) and clawClosed == True {


//            while (rabbit.isBusy()) {
//                telemetry.addData("ServoCmd", starPosition);
//                //telemetry.addData("Turtle Power: ", leftY);
//                telemetry.addData("Rabbit Count: ", rabbit.getCurrentPosition());
//
//                telemetry.update();
//                idle();
//            }


//        myOpMode.telemetry.addData(">", "kkkj");

