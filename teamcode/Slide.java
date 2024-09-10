package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@TeleOp
public class Slide extends LinearOpMode {
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor slideMotor;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode


    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotor.class, "turtle");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        slideMotor.setDirection(DcMotor.Direction.FORWARD);

        // ----------- ENCODER -------------
        // reset encoder counts kept by motors.
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run to target encoder position and stop with brakes on
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set motors to run forward
        slideMotor.setTargetPosition(538);  //538
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        waitForStart();

//        telemetry.addData("Mode", "running");
//        telemetry.addData("encoder", slideMotor.getCurrentPosition() + "something");
//        telemetry.update();
        slideMotor.setPower(0.4);

        while (slideMotor.isBusy()){
            telemetry.addData("Motor", "running!!!!!");


        }
        slideMotor.setPower(0);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
//        slideMotor.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            double leftY = -gamepad1.left_stick_y;
            double slideMotorPower = leftY;

            telemetry.addData("counts", slideMotor.getCurrentPosition());
            telemetry.update();

            slideMotor.setPower(slideMotorPower);

        telemetry.update();

    }


}}


