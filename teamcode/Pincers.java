package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@TeleOp
public class Pincers extends LinearOpMode {
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.


    private Servo leftPincer;
    private Servo rightPincer;
    private double position;


    public static final double SERVO_CENTER = 0.5, SERVO_MIN = 0.0, SERVO_MAX = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {

        leftPincer = hardwareMap.get(Servo.class, "leftPincer");
        rightPincer = hardwareMap.get(Servo.class, "rightPincer");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        position = SERVO_CENTER;

        leftPincer.setPosition(position);
        rightPincer.setPosition(position);

        leftPincer.setDirection(Servo.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.a) {
                position = SERVO_CENTER;
            }

            if (gamepad1.x) {
                if (position < 1) {
                    position += 0.001;
                }
            }


            if (gamepad1.y) {
                if (position > 0) {
                    position -= 0.001;
                }
            }

            leftPincer.setPosition(position);
            rightPincer.setPosition(position);
//            rightPincer.setPosition(1 - position);

            telemetry.addData("LeftServo", position);
            telemetry.update();

            //-------pixel pick-up test-------

//            boolean clawClosed = False;


        }
    }


}


