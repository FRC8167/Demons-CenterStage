/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.DemonPincers;

import java.util.List;
//import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class orig_RobotHardware {


    /* Declare OpMode members. */
     private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
//     public HardwareMap hardwareMap;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    public Servo   leftPincer;
    public Servo   rightPincer;

    public Servo wristServo;

    public DcMotorEx armPivot;
//
    public DcMotorEx armSlide;


    public Servo droneServo;

    public Servo climberServo;

    private List<LynxModule> ctrlHubs;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    public double SERVO_CENTER = 0.5, SERVO_MIN = 0.0, SERVO_MAX = 1.0;
    public double PINCER_MAX = 0.36;

    public double PIVOT_ENC_POWER    =  1.0 ;
    public double PIVOT_HOLD_POWER = 0.05;  //adjust via testing or use PID controller
    public double PINCER_INCR = 0.01;  //sets rate to move/increment servos; seems slow DMW
    public double SLIDE_ENC_POWER = 0.2 ;  //is this used?
    public double CLIMBER_INCR = 0.02;


    //pivot constants
    public int PIVOT_NEST = 0;

    public int PIVOT_STRAIGHT = 1200;  //close to straight DMW  11-15-2023
    public int PIVOT_HIGH = 1473;  //can go higher but set here for safety DMW 11-14-2023
    public int SLIDE_NEST = 20;  //can go to zero but set here for safety DMW 11-14-2023
    public int SLIDE_MEDIUM = 1464;  //placeholder only  ******NOT TESTED******
    public int SLIDE_FULL_EXT = 2000;  //placeholder only  *****NOT TESTED*****
    //other constants
    public int ENCODER_COUNTS_PER_REVOLUTION = 538;  //depends on dcMotor type; is this used??

    public int trgtPos = 0;
    public boolean inMethod = false;

    private double kpPivot = 12;
    private double kpSlide = 8;
    /** Travel Limits and Control Params **/
    private final int PIV_POS_LIMIT_LOW = 0;
    private final int PIV_POS_LIMIT_HIGH = 2200;

    /** Travel Limits and Control Params **/
    private final int SLIDE_RETRACT_LIMIT = -75;
    private final int SLIDE_EXTEND_LIMIT = 1675;

    /** PIVOT Preset Levels **/
    public final int PIVOT_POS_NEST = 20;
    public final int PIVOT_POS_FLOOR_PICKUP = 200;
    public final int PIVOT_POS_SCORE_LOW = 1050;
    public final int PIVOT_POS_SCORE_HIGH = 2000;
    public final int PIVOT_POS_TRAVEL = 1080;

    /** SLIDE Preset Levels **/
    public final int SLIDE_POS_NEST = 20;
    public final int SLIDE_POS_FLOOR_PICKUP = 300;
    public final int SLIDE_POS_SCORE_LOW = 493;
    public final int SLIDE_POS_SCORE_HIGH = 1500;
    public final int SLIDE_POS_TRAVEL = 40;

    /** WRIST SERVO Preset Levels **/
    public final double SERVO_NEST = 0.95;
    public final double SERVO_FLOOR_PICKUP = 0.8628;
    public final double SERVO_SCORE_LOW = 0.8844;
    public final double SERVO_SCORE_HIGH = 0.8588;
    public final double SERVO_FLOOR_DROP = .8744;

    public orig_RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
        init(); // saturday
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init(){
//    public void init(final HardwareMap hwMap)    {
//
        //this.hardwareMap = hwMap;
        myOpMode.telemetry.addData("Status", "detecting...");
        // Define and Initialize Motors (note: need to use reference to actual OpMode).


        // Control Hub
        // Port 0 -
        // Port 1 -
        // Port 2 -
        // Port 3 -
        // Servo 0 -
        // Servo 1 -
        // Servo 2 -
        // Servo 5 -

        // Expansion Hub
        // Port 0 - armPivot
        // Port 1 - armSlide
        // Servo 0 - Drone Launch

        leftPincer = myOpMode.hardwareMap.get(Servo.class, "leftPincer");
        rightPincer = myOpMode.hardwareMap.get(Servo.class, "rightPincer");
        droneServo = myOpMode.hardwareMap.get(Servo.class, "droneServo");
        wristServo = myOpMode.hardwareMap.get(Servo.class, "wristServo");
        climberServo = myOpMode.hardwareMap.get(Servo.class, "climbServo");

        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = myOpMode.hardwareMap.get(DcMotor.class, "rightRear");
        armSlide = myOpMode.hardwareMap.get(DcMotorEx.class, "armSlide");
        armPivot = myOpMode.hardwareMap.get(DcMotorEx.class, "armPivot");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
//
        armPivot.setDirection(DcMotorEx.Direction.REVERSE);  //reversed for encoders/then changed gamepad stick negative to counteract physical direction
        //armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setDirection(DcMotorEx.Direction.REVERSE);  //reversed for encoiders/then changed gamepad stick negative to counteract physical direction
//        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //Notice class is DcMotorEx
        armPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ctrlHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


//
//        //**************************************
//
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        armPivot.setPower(0);
        armSlide.setPower(0);

        //Define and Initialize Servos (use actual OpMode name)

        leftPincer.setDirection(Servo.Direction.REVERSE);
        leftPincer.setPosition(0.35);
        rightPincer.setPosition(0.35);

        droneServo.setPosition(0.65);
        wristServo.setPosition(0.95);
        climberServo.setPosition(0.425);

        setPidPivot(kpPivot);
        setPidSlide(kpSlide);

//        myOpMode.telemetry.addData(">", "Hardware Initialized");
//        myOpMode.telemetry.update();
    }

    public void resetArmSystemEncoders() {
        armPivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPidPivot(double pivotpGain) {
        kpPivot = pivotpGain;
        armPivot.setPositionPIDFCoefficients(kpPivot);
    }

    public void setPidSlide(double slidepGain) {
        kpSlide = slidepGain;
        armSlide.setPositionPIDFCoefficients(kpSlide);
    }



    public Boolean isOpen = true;
    public Boolean isOpenLeft = true;
    public Boolean isOpenRight = true;
    public Boolean isOpenClimber = true;

    public void openPincer() {
        if (isOpen == false) {
            leftPincer.setPosition(0.6);
            rightPincer.setPosition(0.6);
            isOpen = true;
        }
    }

    public void closePincer() {
        if (isOpen == true) {
            leftPincer.setPosition(0.35);
            rightPincer.setPosition(0.35);
            isOpen = false;
        }
    }

    public void climberToggle() {
        if (isOpenClimber == true) {
            climberServo.setPosition(0.0);
            isOpenClimber = false;
        }
        else{
            climberServo.setPosition(1.0);
            isOpenClimber = true;
        }
    }


    //manually control servos
    public void manualPincer(double newPosition){
        leftPincer.setPosition(newPosition);
        rightPincer.setPosition(newPosition);
    }

    public void manualClimber(double newPosition){
        climberServo.setPosition(newPosition);
    }


    public void togglePincer() {
        if (isOpenLeft == true && isOpenRight == true) {
            leftPincer.setPosition(0.35);
            rightPincer.setPosition(0.35);
            isOpenLeft = false;
            isOpenRight = false;
        }
        else{
                leftPincer.setPosition(0.6);
                rightPincer.setPosition(0.6);
                isOpenLeft = true;
                isOpenRight = true;
            }
        }

    public void leftToggle() {
        if (isOpenLeft == true) {
            leftPincer.setPosition(0.35);
            isOpenLeft = false;
        }

        else{
            leftPincer.setPosition(0.6);
            isOpenLeft = true;
        }
    }

    public void rightToggle() {
        if (isOpenRight == true) {
            rightPincer.setPosition(0.35);
            isOpenRight = false;
        }
        else{
            rightPincer.setPosition(0.6);
            isOpenRight = true;
        }
    }

        //Toggle the stat of both servos
//        if (isOpen) {
//            closePincer();
//        } else {
//            openPincer();
//        }


    public boolean launcherReady = true;
    public void launchDrone(){
        droneServo.setPosition(1.0);
        launcherReady = false;
    }
    public void resetDroneLauncher(){
         droneServo.setPosition(0.6);
         launcherReady = true;
    }
//
    public void manualPivot(double pivotPower)   {
        armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPivot.setPower(pivotPower);
    }

    public void manualWrist(double position) {
        wristServo.setPosition(position);
    }

    public void stopArmWithHold(){
        double currentPosition = armPivot.getCurrentPosition();
        double holdPower = PIVOT_HOLD_POWER;

        //hold straight out
        if (currentPosition < PIVOT_STRAIGHT && currentPosition > PIVOT_NEST) {
            armPivot.setPower(holdPower);
        } else if (currentPosition> PIVOT_STRAIGHT && currentPosition < PIVOT_HIGH) {
            armPivot.setPower(holdPower*0.8);  //just messing with the code
        }
        else {
            armPivot.setPower(0);
        }
    }

     //**************************************************************************//
     public void encoderPivot(int pivotTargetPosition) {
         trgtPos = pivotTargetPosition;  //what does this do?  You do not call it
//             armPivot.getCurrentPosition();
//             armPivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
         armPivot.setTargetPosition(trgtPos);
         armPivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
         armPivot.setPower(0.85);

//             while (armPivot.isBusy()) {  ///hmmmm
//             }
//             inMethod = false;
     }

    //**************************************************************************//
    public void stopPivot(){
        armPivot.setPower(0);
    }
//
    public void manualSlide(double slidePower) {
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setPower(slidePower);
    }
    public void encoderSlide(int slideTargetPosition){
        trgtPos = slideTargetPosition;  //what does this do?  You do not call it
        armSlide.getCurrentPosition();
//        armSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armSlide.setTargetPosition(trgtPos);
        armSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armSlide.setPower(0.85);  //test what is efficient
    }
    public void stopSlide(){
        armSlide.setPower(0);
    }

    public void mecanumDrive(double Drive, double Strafe, double Turn) {
        double y = Drive;
        double x = Strafe;
        double rx = Turn * 0.75;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y+x+rx) / denom;
        double backLeftPower = (y-x+rx) / denom;
        double frontRightPower = (y-x-rx) / denom;
        double backRightPower = (y+x-rx) / denom;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftRear.setPower(backLeftPower);
        rightRear.setPower(backRightPower);
    }

    public void openLeftPincer() {
        leftPincer.setPosition(0.6); //.35
    }
    public void closeLeftPincer() {
        leftPincer.setPosition(0.35);
    }
    public void openRightPincer() {
        rightPincer.setPosition(0.6);
    }
    public void closeRightPincer() {
        rightPincer.setPosition(0.35);
    }


//    public double hangServoCurrent(){
//        return ctrlHubs.get(0).getCurrent(CurrentUnit.MILLIAMPS);
//    }



    /*************************************************************************/

    public int pivotClampPositionSetpt(int pivotSetpoint) {
        if (pivotSetpoint > PIV_POS_LIMIT_HIGH) {
            pivotSetpoint = PIV_POS_LIMIT_HIGH;
        } else if (pivotSetpoint < PIV_POS_LIMIT_LOW) {
            pivotSetpoint = PIV_POS_LIMIT_LOW    ;
        }
        return pivotSetpoint;
    }


    /*************************************************************************/
    public int slideClampPositionSetpt(int slideSetpoint) {
        if (slideSetpoint > SLIDE_EXTEND_LIMIT) {
            slideSetpoint = SLIDE_EXTEND_LIMIT;
        } else if (slideSetpoint < SLIDE_RETRACT_LIMIT) {
            slideSetpoint = SLIDE_RETRACT_LIMIT;
        }
        return slideSetpoint;
    }

    /*************************************************************************/



    public void pivotMoveToCountPosition(int trgtPosCnts, double duration_sec) {

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

        armPivot.setTargetPositionTolerance(40);
        int currentPosition = armPivot.getCurrentPosition();
        double rate = (trgtPosCnts - currentPosition) / duration_sec;

        armPivot.setTargetPosition(pivotClampPositionSetpt(trgtPosCnts));
        armPivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armPivot.setVelocity(rate);

    }

    /*************************************************************************/
    public void slideMoveToCountPosition(int trgtPosCnts, double duration_sec) {

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
        armSlide.setTargetPositionTolerance(40);

        int currentPosition = armSlide.getCurrentPosition();
        double rate = (trgtPosCnts - currentPosition) / duration_sec;

        armSlide.setTargetPosition(slideClampPositionSetpt(trgtPosCnts));
        armSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armSlide.setVelocity(rate);

    }


}



