package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.teamcode.Utilities.BinaryControl;

@TeleOp(name="wheatTeleOp", group="Experimental")
@Disabled
public class wheatTeleOp extends RobotBase_Demons {

    public enum LiftState {
        LIFT_IDLE,
        LIFT_EXTEND,
        LIFT_DROP,
        LIFT_NEST,
        Lift_NEST_COMPLETE
    }

    public enum Action {
        SCORE,
        ACQUIRE
    }

    public enum Score {
        LEFT,
        RIGHT,
        BOTH
    }

    /**
     * Change these if used in Autonomous
     **/
    Action action = Action.ACQUIRE;        // Change to score for Autonomous if starting with Pixel
    LiftState state = LiftState.LIFT_IDLE;
    Score score = Score.BOTH;

    ElapsedTime liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//    Gamepad gp1 = gamepad1;
//    Gamepad gp2 = gamepad2;
//
//    BinaryControl leftPincerToggle = new BinaryControl(gamepad2.left_bumper);
//    BinaryControl rightPincerToggle = new BinaryControl(gamepad2.right_bumper);
//    BinaryControl shootLeftApproach = new BinaryControl(gamepad1.b);


    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(); // Calls super initHardware (from RobotBase_Demons)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** Initialization should take place in an auto and not overwritten here **/
        pivotJoint.resetEncoder();
        slideSys.resetEncoder();
        /**----------------------------------------------------------------------**/

//        teamAlliance = AllianceColor.BLUE;

        vision.disableTfodDetection();
        vision.enableAprilTagDetection();

        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        BinaryControl leftPincerToggle = new BinaryControl(gamepad2.left_bumper);
        BinaryControl rightPincerToggle = new BinaryControl(gamepad2.right_bumper);
        BinaryControl shootLeftApproach = new BinaryControl(gamepad1.b);

        waitForStart();

        liftTimer.reset();

        while (opModeIsActive()) {

            gp1 = gamepad1;
            gp2 = gamepad2;

            leftPincerToggle.update(gp2.left_bumper);
            rightPincerToggle.update(gp2.right_bumper);
            shootLeftApproach.update(gp1.b);

            /** Are April Tags Available? **/
            while(gp1.b) {
//                if (vision.aprilTagsDetected()) {
////                    telemetry.addData("April Tags Detected: ", vision.aprilTagCount());
//                    /** Check if driver wants auto and left tag available **/
//
////                    for (AprilTagDetection detection : vision.tagsInView()) {
//                        if (detection.metadata != null) {
//                            //  Check to see if we want to track towards this tag.
//                            if (gp1.b && (detection.id == vision.BLUE_RIGHT)) {
//                                drive.shootAprilTagApproach(detection, 18);
//                            }
//                        }
//                    }
//                }
            }

            /** Read Joystick for Drive Motions **/
            telemetry.addData("April Tags Detected: ", 0);
            drive.mecanumDrive(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);

            /** Check for Drone Launch Commands **/
            if (gp1.left_bumper && gp1.x) {
                drone.launchDrone();
            }
            if (gp1.left_bumper && gp1.y) {
                drone.resetDroneLauncher();
            }

            /** Check for Auto Movements **/
            switch (state) {

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIFT_IDLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                case LIFT_IDLE:

                    /** Check for Manual Pincer Commands **/
                    if (leftPincerToggle.state()) pincers.toggleLeftPincer();
                    if (rightPincerToggle.state()) pincers.toggleRightPincer();

                    /** Check for Manual Pivot Rotate **/
                    pivotJoint.manualPivotRotate(0.7 * gp2.left_stick_y);

                    /** Check for Manual Slide Extend/Retract **/
                    slideSys.manualSlideMove(-0.5 * gp2.right_stick_y);

                    /** Check for Manual Wrist Rotate **/
                    if (gp2.left_trigger > 0.5) {
                        wristJoint.setPosition(wristJoint.getServoPos() + 0.1);
                    }

                    if (gp2.right_trigger > 0.5) {
                        wristJoint.setPosition(wristJoint.getServoPos() + 0.1);
                    }

                    /** Start a set of tasks to complete an action **/
                    if (gp2.a) {                                         /** Position Floor Pickup **/
                        pivotJoint.moveToCountPosition(pivotJoint.POS_FLOOR_PICKUP, 0.25);
                        pincers.openPincers();
                        slideSys.moveToCountPosition(slideSys.POS_FLOOR_PICKUP, 0.5);
                        wristJoint.setPosition(wristJoint.POS_FLOOR_PICKUP);
                        action = Action.ACQUIRE;
                        state = LiftState.LIFT_EXTEND;
                    } else if (gp2.b) {                                  /** Position Score Low **/
                        pivotJoint.moveToCountPosition(pivotJoint.POS_SCORE_LOW, 0.25);
                        slideSys.moveToCountPosition(slideSys.POS_SCORE_LOW, .5);
//                        wristJoint.setPosition(wristJoint.POS_SCORE_LOW);
                        action = Action.SCORE;
                        state = LiftState.LIFT_EXTEND;
                    } else if (gp2.y) {                                  /** Position Score High **/
                        pivotJoint.moveToCountPosition(pivotJoint.POS_SCORE_HIGH, 0.75);
                        wristJoint.setPosition(wristJoint.POS_SCORE_HIGH);
                        slideSys.moveToCountPosition(slideSys.POS_SCORE_HIGH, 1);
                        action = Action.SCORE;
                        state = LiftState.LIFT_EXTEND;
                    } else if (gp2.x) {                                  /** Position Nest **/
                        action = Action.ACQUIRE;
                        state = LiftState.LIFT_DROP;
                    }
                    liftTimer.reset();
                    break;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIFT_EXTEND ~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                case LIFT_EXTEND:
                    if (liftTimer.milliseconds() > 50) {
                        /* 50ms required for busy bits to set in motor controllers */
                        if (pivotJoint.motionFinished() && slideSys.motionFinished()) {     // Add timeout? liftTimer > ?
                            state = LiftState.LIFT_DROP;

                            /****/sleep(500); /****/

                        }
                        liftTimer.reset();
                    }
                    break;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIFT_DROP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                case LIFT_DROP:
                    pincers.openPincers();
                    switch (action) {
                        case SCORE:
                            if (liftTimer.milliseconds() > 1000) {
                                state = LiftState.LIFT_NEST;     // Allow 1.1s for servo pincers to open
                                liftTimer.reset();
                            }
                            break;

                        case ACQUIRE:
                            // Motion is complete for Pick up off of floor, Return to idle
                            state = LiftState.LIFT_IDLE;
                            break;
                    }
                    break;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIFT_NEST ~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                case LIFT_NEST:
                    slideSys.moveToCountPosition(slideSys.POS_NEST, 0.25);
                    wristJoint.setPosition(wristJoint.POS_NEST);
                    pivotJoint.moveToCountPosition(pivotJoint.POS_NEST, 0.75);
                    pincers.closePincers();

                    state = LiftState.Lift_NEST_COMPLETE;
                    liftTimer.reset();
                    break;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~ LIFT_NEST_COMPLETE ~~~~~~~~~~~~~~~~~~~~~~ */
                case Lift_NEST_COMPLETE:
                    if (pivotJoint.motionFinished() && slideSys.motionFinished()) {     // Add timeout? liftTimer > ?
                        state = LiftState.LIFT_IDLE;
                    }
                    break;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DEFAULT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                default:
                    state = LiftState.LIFT_IDLE;
                    break;
            }

            /** Check for limits tripped in a manual mode **/
            pivotJoint.update();
            slideSys.update();
            wristJoint.update();

            /** Update Telemetry **/
            telemetry.addLine("System");
            telemetry.addData("State Machine: ", state.toString());
            telemetry.addLine(" ");
            telemetry.addData("Battery Voltage [V]: ", battVoltage());
//            telemetry.addData("Total Current Draw [mA]: ", totalCurrentmA());

            telemetry.addLine("\nPivot System");
            telemetry.addData("Position [cnt]: ", pivotJoint.getPivotPos());
//            telemetry.addData("Power: ", decPlace2(pivotJoint.getPivotPower()));
//            telemetry.addData("Motor Current [A]: ", decPlace2(pivotJoint.getPivotCurrent()));

            telemetry.addLine("\nSlide System");
            telemetry.addData("Position [cnt]: ", slideSys.getSlidePos());
//            telemetry.addData("Power: ", decPlace2(slideSys.getPivotPower()));

            telemetry.addLine("\nWrist System");
            telemetry.addData("Position", wristJoint.getServoPos());

            telemetry.update();

        }
    }
}



/** GAME CONTROLLER 2 **
 *  A                   Floor Pickup
 *  B                   Score Low
 *  X                   Nest
 *  Y                   Score High
 *  LEFT TRIGGER        Manual Wrist Up
 *  RIGHT TRIGGER       Manual Wrist Down
 *  LEFT BUMPER         Launch Drone
 *  RIGHT BUMPER        Reset Drone
 *  DPAD UP
 *  DPAD DOWN
 *  DPAD LEFT
 *  DPAD RIGHT
 *  BACK
 *  START
 *  LEFT JOYSTICK X
 *  LEFT JOYSTICK Y     Pivot Angle
 *  RIGHT JOYSTICK X
 *  RIGHT JOYSTICK Y    Slide Extend / Retract
 */

/** State Machine
 *  States: Idle, Nest Position, Pickup Position, Drop Position, Score High, Score Low
 *
 *  Idle - Do no action
 *
 *  Nest to Drop -> on button or Auto
 *  Drop to Next on Claw open
 *
 *  Next to Pickup -> on button press
 *  Pickup to Nest -> on claw close
 *
 *  Nest to Score High -> on button press
 *  Return to Nest -> Claw Open (close claw and return)
 *
 *  Above for low scoring - different set point
 *
 */
