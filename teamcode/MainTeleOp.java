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

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective could put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="MainTeleOp", group="Robot")
public class MainTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.

    private double snail = 0.7;
    private boolean snailMode = false;
    private boolean presetInProgress = false;

    private double hub0Current;
    private double maxCurrentmA = 0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.resetArmSystemEncoders();

        BinaryControl leftPincerToggle = new BinaryControl(gamepad2.left_bumper);
        BinaryControl rightPincerToggle = new BinaryControl(gamepad2.right_bumper);
        BinaryControl wristUp = new BinaryControl(gamepad2.dpad_up);
        BinaryControl wristDown = new BinaryControl(gamepad2.dpad_down);
        BinaryControl lowAuto = new BinaryControl(gamepad2.b);

        boolean climb = false;

        //HardwareMap hardwareMap = new HardwareMap(this.hardwareMap, this);
//        double drive        = 0;
//        double strafe       = 0;
//        double turn         = 0;
//        double arm          = 0;
//        double handOffset   = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
//        robot.init();
//        robot.armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftPincerToggle.update(gamepad2.left_bumper);
            rightPincerToggle.update(gamepad2.right_bumper);
            wristUp.update(gamepad2.dpad_up);
            wristDown.update(gamepad2.dpad_down);
            lowAuto.update(gamepad2.b);

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            if (gamepad1.right_bumper) {
                snail = .4;
            } else snail = 0.7;

            double drive = -gamepad1.left_stick_y * snail;
            double strafe = gamepad1.left_stick_x * snail;
            double turn = gamepad1.right_stick_x * snail;


            // Call the Mecanum Drive method from the RobotHardware class
            robot.mecanumDrive(drive, strafe, turn);


            if (wristUp.state()) {
                double upWristPosition = robot.wristServo.getPosition();
                if (upWristPosition < 1) {
                    upWristPosition += 0.009;
                }
                robot.manualWrist(upWristPosition);
            }


            if (wristDown.state()) {
                double wristPosition =
                        robot.wristServo.getPosition();
                if (wristPosition > 0) {
                    wristPosition -= 0.009;
                }
                robot.manualWrist(wristPosition);
            }


            if (leftPincerToggle.state()) robot.rightToggle();

            if (rightPincerToggle.state()) robot.leftToggle();


            //pixel pickup from floor
            if (gamepad2.a) {
                presetInProgress = true;
                robot.pivotMoveToCountPosition(200, 0.5);
//                robot.wristServo.setPosition(0.8572);             // .45
                robot.wristServo.setPosition(robot.SERVO_FLOOR_PICKUP);             // .45
                robot.slideMoveToCountPosition(400, 0.5);
            }

            //score low position
            if(lowAuto.state()) {
                presetInProgress = true;
                robot.pivotMoveToCountPosition(957, 0.5);
                robot.slideMoveToCountPosition(400, 0.5);
                robot.wristServo.setPosition(robot.SERVO_SCORE_LOW);             // .45
            }

            //score high position - Adjust values
//            if (gamepad2.y) {
//                presetInProgress = true;
//                robot.pivotMoveToCountPosition(1500, 0.5);
//                robot.slideMoveToCountPosition(1600, 0.5);
////                robot.wristServo.setPosition(0.8588);                           //.45
//                robot.wristServo.setPosition(robot.SERVO_SCORE_HIGH);
//            }

            //travel under bar
            if (gamepad2.x) {
                presetInProgress = true;
                robot.pivotMoveToCountPosition(820, 0.5);
                robot.slideMoveToCountPosition(50, 0.5);
                robot.wristServo.setPosition(0.95);
            }


            if (gamepad1.left_bumper && gamepad1.x) {
                robot.launchDrone();
            }
            if (gamepad1.left_bumper && gamepad1.y) {
                robot.resetDroneLauncher();
            }

            if(!presetInProgress) {
                double pivotPower = -gamepad2.left_stick_y;  //should || be && ????
                if (robot.armPivot.getCurrentPosition() < robot.PIVOT_HIGH ||
                        robot.armPivot.getCurrentPosition() > robot.PIVOT_NEST) {
                    robot.manualPivot(0.7 * pivotPower);
                } else {
                    robot.stopPivot();
                }

                double slidePower = -0.5 * gamepad2.right_stick_y;
                if (robot.armSlide.getCurrentPosition() < robot.SLIDE_FULL_EXT ||
                        robot.armSlide.getCurrentPosition() > robot.SLIDE_NEST) {
                    robot.manualSlide(slidePower);
                } else {
                    robot.stopSlide();
                }
            }


            //DISABLE CLIMBER SERVO
            if (gamepad1.a) {
                ((ServoImplEx) robot.climberServo).setPwmDisable();
            }

            //RE-ENABLE CLIMBER SERVO
            if (gamepad1.b) {
                ((ServoImplEx) robot.climberServo).setPwmEnable();
            }


//            while (gamepad1.left_trigger > 0) {
            if (gamepad1.left_trigger > 0) {
                double position = robot.climberServo.getPosition();
                if (position > 0) {
                    position -= robot.CLIMBER_INCR;
                }
                robot.climberServo.setPosition(position);
                sleep(50);
            }

//            while (gamepad1.right_trigger > 0) {
            if(gamepad1.right_trigger > 0) {
                double position = robot.climberServo.getPosition();
                if (position < 1) {
                    position += robot.CLIMBER_INCR;
                }
                robot.climberServo.setPosition(position);
                sleep(50);
            }


            if (gamepad2.dpad_left) {
                climb = true;
            }
            while (climb == true) {
                robot.manualPivot(-0.5);
                if (gamepad2.dpad_right) {
                    climb = false;
                    break;
                }
            }

            if(Math.abs(robot.armSlide.getCurrentPosition() - robot.armSlide.getTargetPosition()) < 40 &&
                    Math.abs(robot.armPivot.getCurrentPosition() - robot.armPivot.getTargetPosition()) < 40) {
                presetInProgress = false;
            }

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick Y-val");
            telemetry.addData("Strafe", "Left Stick X-val");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Drive Power", "%.2f", -gamepad1.left_stick_y);
            telemetry.addData("Strafe Power", "%.2f", gamepad1.left_stick_x * 1.1);
            telemetry.addData("Turn Power", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("ManualPivotPower", -gamepad2.left_stick_y);
            telemetry.addData("Pivot Encoder Value:  ", robot.armPivot.getCurrentPosition());
            telemetry.addData("SlidePower", -gamepad2.right_stick_y);
            telemetry.addData("ManualSlide Encoder Value:  ", robot.armSlide.getCurrentPosition());
            telemetry.addData("Target Position: ", robot.trgtPos);
//            telemetry.addData("In Loop :", robot.inMethod);
            telemetry.addData("Wrist Position", robot.wristServo.getPosition());
//            telemetry.addData("Slide Busy", robot.armSlide.getTargetPosition());
            telemetry.addData("Slide Target", robot.armSlide.getTargetPosition());
//            telemetry.addData("\nCtrl Hub Current[mA]: ", hub0Current);
//            telemetry.addData("Ctrl Hub Max I[mA]: ", maxCurrentmA);

            if (snail > .4) {
                snailMode = false;
            } else snailMode = true;
            telemetry.addData("Snail Engaged", snailMode);

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
//            sleep(50);
        }
    }
}




