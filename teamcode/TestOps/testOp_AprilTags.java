package org.firstinspires.ftc.teamcode.TestOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotBase_Demons;
import org.firstinspires.ftc.teamcode.Utilities.AprilTagApproach;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Disabled
@TeleOp(name="TestAprilTagsV4", group="Test")
public class testOp_AprilTags extends RobotBase_Demons {

    /** Rates **/
    public static double acqHz  = 40;    // 200Hz sample rate ~25ms to process an image;
    public static double ctrlHz = 40;    // 10Hz control rate (update controls 10 times per sec)
    public static double filterCutoffHz = 10.0;
    public static double xOverDistance  = 100;

    public static double[] driveCmds = {0,0,0};

    public static double RANGE_OFFSET   = 16;
    public static double LATERAL_OFFSET = 2;
    public static double YAW_OFFSET     = 2;

    public static double KP_DRIVE  = 0.020; //0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double KP_STRAFE = 0.020; //0.07;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double KP_YAW    = 0.004; //0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double KD_DRIVE  = 0.001;
    public static double KD_STRAFE = 0.001;
    public static double KD_YAW    = 0.001;

    public static double KF_DRIVE  = 0.05;
    public static double KF_STRAFE = 0.08; //0.12;
    public static double KF_YAW    = 0.09; //0.10;

    private static double MAX_AUTO_SPEED  = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    private static double MAX_AUTO_STRAFE = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    private static double MAX_AUTO_TURN   = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    private int tagsInView = 0;
    private double x, y, yaw, range, bearing;
    private double[] lastTargetErr = {0, 0, 0};
    private double deltaT = 1;
    private List<AprilTagDetection> tagsIdentified;

//    CameraIntrinsics cam2Intrinsics = new CameraIntrinsics(782.7129, 782.7129, 279.9392,242.9770);


    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(); // Calls super initHardware (from RobotBase_Demons)

        // Class Objects
//        ElapsedTime acqTimer  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime ctrlTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//        Vision vision = new Vision(webCam, cam2Intrinsics);

//        AprilTagApproach aprilTag = new AprilTagApproach(ctrlHz);
        AprilTagApproach aprilTag = new AprilTagApproach();

        sleep(500);                                  // Wait for camera to initialize
        vision.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        /** Initialization should take place in an auto and not overwritten here **/
        /** This is for teleOp evaluation/debug use only                         **/

        setAlliance(Alliance.BLUE);
        /**----------------------------------------------------------------------**/

        vision.enableAprilTagDetection();

        waitForStart();

        /** ................................................................... **/
        while (opModeIsActive()) {

            deltaT = 1/ctrlHz;

            /** Acquire Sensor Data **/
//            if(acqTimer.milliseconds() > 1/acqHz) {
//                acqTimer.reset();
                vision.scanForAprilTags();
                tagsInView = vision.aprilTagsDetectedCount();
                tagsIdentified = vision.allAprilTagsDetected();
//            }

            /** Tags available and driver wants to shoot and automatic approach **/
            if (ctrlTimer.milliseconds() > 1/ctrlHz) {
                ctrlTimer.reset();
                if(gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                    if (vision.aprilTagAvailable(vision.BLUE_LEFT) && (gamepad1.dpad_left)) {
                        assignLocationVars(vision.getTagData(vision.BLUE_LEFT));
                        if(gamepad1.right_bumper) {
                            driveCmds = aprilTag.shootRBHApproach(vision.getTagData(vision.BLUE_LEFT));
                        } else {
                            if(gamepad1.left_bumper) driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_LEFT));
                        }
                    }

                    if (vision.aprilTagAvailable(vision.BLUE_CENTER) && (gamepad1.dpad_up)) {
                        assignLocationVars(vision.getTagData(vision.BLUE_CENTER));
                        if(gamepad1.right_bumper) {
                            driveCmds = aprilTag.shootRBHApproach(vision.getTagData(vision.BLUE_CENTER));
                        } else {
                            if(gamepad1.left_bumper) driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_CENTER));
                        }
                    }

                    if (vision.aprilTagAvailable(vision.BLUE_RIGHT) && (gamepad1.dpad_right)) {
                        assignLocationVars(vision.getTagData(vision.BLUE_RIGHT));
                        if(gamepad1.right_bumper) {
                            driveCmds = aprilTag.shootRBHApproach(vision.getTagData(vision.BLUE_RIGHT));
                        } else {
                            if(gamepad1.left_bumper) driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_RIGHT));
                        }
                    }

                    drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);  // Full motion

                } else {
                    driveCmds[0] = -gamepad1.left_stick_y * .6;
                    driveCmds[1] =  gamepad1.left_stick_x * .6;
                    driveCmds[2] =  gamepad1.right_stick_x * .6;
                    drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);
                }
            }

            telemetry.addData("Tags in View:", tagsInView);
            for(AprilTagDetection tag : tagsIdentified) {
//                telemetry.addData("Id: ", tag.id);
                telemetry.addData("Target", "ID %d (%s)", tag.id, tag.metadata.name);
            }

            double drvcmd = driveCmds[0] * 100;
            double strcmd = driveCmds[1] * 100;
            double thecmd = driveCmds[2] * 100;

            telemetry.addLine("\n--------- Target Tag Data ----------");
            telemetry.addData("Tag Y X Yaw", "%5.2f, %5.2f, %5.2f", y, x, yaw);
            telemetry.addData("Tag Range Bearing", "%5.2f, %5.2f", range, bearing);

            telemetry.addLine("\n---------- Drive Commands ----------");
            telemetry.addData("Drive %", " %5.1f",drvcmd);
            telemetry.addData("Strafe %", " %5.1f",strcmd);
            telemetry.addData("Turn %", " %5.1f",thecmd);
            telemetry.update();
        }
    }

//    /*************************************************************************/
//    public double[] shootPidApproach(AprilTagDetection targetTag) {
//        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//        double yError   = targetTag.ftcPose.y   - RANGE_OFFSET;
//        double xError   = targetTag.ftcPose.x   - LATERAL_OFFSET;
//        double yawError = targetTag.ftcPose.yaw - YAW_OFFSET;
//
//        double[] cmds = {0, 0, 0};
//
//        // Calculate PD Commands
//        cmds[0] = KP_DRIVE  * yError + KD_DRIVE  * (yError - lastTargetErr[0]) / deltaT; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));;
//
//        /** Calculate Strafe Command **/
//        cmds[1] = KP_STRAFE * xError + KD_STRAFE * (xError - lastTargetErr[1]) / deltaT;// + KF_STRAFE;
//        if(xError > 0) { cmds[1] += KF_STRAFE; } else { cmds[1] -= KF_STRAFE; }
//
//        /** Calculate Yaw Command **/
//        cmds[2] = KP_YAW * yawError + KD_YAW * (yawError - lastTargetErr[2]) / deltaT; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));
//        if(yawError > 0) { cmds[2] += KF_YAW; } else { cmds[2] -= KF_YAW; }
//
//        lastTargetErr[0] = yError;
//        lastTargetErr[1] = xError;
//        lastTargetErr[2] = yawError;
//
//        // Use the speed and turn "gains" to calculate how we want the robot to move.
//        cmds[0] = Range.clip(cmds[0], -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
//        cmds[1] = Range.clip(cmds[1], -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        cmds[2] = Range.clip(-cmds[2], -MAX_AUTO_TURN,   MAX_AUTO_TURN);
//
//        return cmds;
//    }

//    /*************************************************************************/
//    private double[] farApproach(AprilTagDetection tag) {
//
//        double rangeError   = tag.ftcPose.range   - RANGE_OFFSET;
//        double headingError = tag.ftcPose.bearing - LATERAL_OFFSET;
//        double yawError     = tag.ftcPose.yaw     - YAW_OFFSET;
//
//        double[] cmds = {0.0, 0.0, 0.0};
//
//        // Calculate PD Commands
//        cmds[0] = KP_DRIVE  * rangeError   + KD_DRIVE  * (rangeError   - lastTargetErr[0]) / deltaT;
//        cmds[1] = KP_STRAFE * yawError     + KD_STRAFE * (headingError - lastTargetErr[1]) / deltaT;
//        cmds[2] = KP_YAW    * headingError + KD_YAW    * (yawError     - lastTargetErr[2]) / deltaT;
//
//        lastTargetErr[0] = rangeError;
//        lastTargetErr[1] = headingError;
//        lastTargetErr[2] = yawError;
//
//        return cmds;
//    }
//
//    /*************************************************************************/
//    private double[] nearApproach(AprilTagDetection tag) {
//
//        double yError   = tag.ftcPose.y   - RANGE_OFFSET;
//        double xError   = tag.ftcPose.x   - LATERAL_OFFSET;
//        double yawError = tag.ftcPose.yaw - YAW_OFFSET;
//
//        double[] cmds = {0, 0, 0};
//
//        // Calculate PD Commands
//        cmds[0] = KP_DRIVE * yError + KD_DRIVE * (yError - lastTargetErr[0]) / deltaT;
//        cmds[1] = KP_STRAFE * xError + KD_STRAFE * (xError - lastTargetErr[1]) / deltaT;
//
////	cmds[2] = KP_YAW * yawError + KD_YAW * (yawError - lastTargetErr[2]) / deltaT;
//        cmds[2] = KP_YAW * yawError * Math.cos(Math.toRadians(yError)) + KD_YAW * (yawError - lastTargetErr[2]) / deltaT;
//
//        lastTargetErr[0] = yError;
//        lastTargetErr[1] = xError;
//        lastTargetErr[2] = yawError;
//
//        return cmds;
//    }

    /*************************************************************************/
    private void assignLocationVars(AprilTagDetection tag) {
        x = tag.ftcPose.x;
        y = tag.ftcPose.y;
        yaw = tag.ftcPose.yaw;
        range = tag.ftcPose.range;
        bearing = tag.ftcPose.bearing;
    }
}
