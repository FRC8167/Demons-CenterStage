package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagApproach {

    private double[] lastTargetErr = {0, 0, 0};

    /** Adjust for Camera and Robot Offsets from April Tag **/
    private double RANGE_OFFSET   = 16;
    private double LATERAL_OFFSET = 2;
    private double YAW_OFFSET     = 2;

    /** Drive (y) Axis Gains **/
    private double KP_DRIVE  = 0.020;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private double KD_DRIVE  = 0.001;
    private double KF_DRIVE  = 0.04;

    /** Strafe (x) Axis Gains **/
    private double KP_STRAFE = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    private double KD_STRAFE = 0.001;
    private double KF_STRAFE = 0.04;

    /** Yaw (turn) Axis Gains **/
    private double KP_YAW    = 0.004;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private double KD_YAW    = 0.001;
    private double KF_YAW    = 0.035;

    /** Maximum Speed for Auto Driving **/
    private double MAX_AUTO_SPEED  = 0.40;   //  Clip the approach speed to this max value (adjust for your robot)
    private double MAX_AUTO_STRAFE = 0.40;   //  Clip the approach speed to this max value (adjust for your robot)
    private double MAX_AUTO_TURN   = 0.30;   //  Clip the turn speed to this max value (adjust for your robot)

    private double x, y, yaw, range, bearing;

    private double deltaT;
    private List<AprilTagDetection> tagsIdentified;

//    Vision vision = new Vision(webCam, cam2Intrinsics);
//    sleep(500);                                  // Wait for camera to initialize
//    vision.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
//    AprilTagApproach aprilTag = new AprilTagApproach(ctrlHz);
//
//    vision.scanForAprilTags();
//    tagsInView = vision.aprilTagsDetectedCount();
//    tagsIdentified = vision.allAprilTagsDetected();

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public AprilTagApproach() {
        /** If constructor called without a sampling interval, deltaT must = 1 to avoid a divide by zero
           and D terms in the PID loop must equal zero **/
        deltaT = 1;
        KD_DRIVE  = 0;
        KD_STRAFE = 0;
        KD_YAW    = 0;
    }

    public AprilTagApproach(double ControlHz) {
        deltaT = 1 / ControlHz;
    }
    /** \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\///////////////////////////////// **/


    /*************************************************************************/
    public double[] shootPidApproach(AprilTagDetection targetTag) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double yError   = targetTag.ftcPose.y   - RANGE_OFFSET;
        double xError   = targetTag.ftcPose.x   - LATERAL_OFFSET;
        double yawError = targetTag.ftcPose.yaw - YAW_OFFSET;

        double[] cmds = {0, 0, 0};

        /** Calculate PD Commands **/
        cmds[0] = KP_DRIVE  * yError + KD_DRIVE  * (yError - lastTargetErr[0]) / deltaT; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));;

        /** Calculate Strafe Command **/
        cmds[1] = KP_STRAFE * xError + KD_STRAFE * (xError - lastTargetErr[1]) / deltaT;// + KF_STRAFE;
        if(xError > 0) { cmds[1] += KF_STRAFE; } else { cmds[1] -= KF_STRAFE; }

        /** Calculate Yaw Command **/
        cmds[2] = KP_YAW * yawError + KD_YAW * (yawError - lastTargetErr[2]) / deltaT; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));
        if(yawError > 0) { cmds[2] += KF_YAW; } else { cmds[2] -= KF_YAW; }

        lastTargetErr[0] = yError;
        lastTargetErr[1] = xError;
        lastTargetErr[2] = yawError;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        cmds[0] = Range.clip(cmds[0],  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
        cmds[1] = Range.clip(cmds[1],  -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        cmds[2] = Range.clip(-cmds[2], -MAX_AUTO_TURN,  MAX_AUTO_TURN);

        return cmds;
    }

    /*************************************************************************/
    public double[] shootRBHApproach(AprilTagDetection targetTag) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError   = targetTag.ftcPose.range - RANGE_OFFSET;
        double headingError = targetTag.ftcPose.bearing;
        double yawError     = targetTag.ftcPose.yaw;

        double[] cmds = {0, 0, 0};

        // Calculate PD Commands
        cmds[0] = KP_DRIVE * rangeError; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));;

        /** Calculate Strafe Command **/
        cmds[1] = KP_STRAFE * headingError;// + KF_STRAFE;
        if(headingError > 0) { cmds[1] += KF_STRAFE; } else { cmds[1] -= KF_STRAFE; }

        /** Calculate Yaw Command **/
        cmds[2] = KP_YAW * yawError; // * Math.cos(Range.clip(Math.toRadians(yawError), -Math.PI/2, Math.PI/2));
        if(yawError > 0) { cmds[2] += KF_YAW; } else { cmds[2] -= KF_YAW; }

        lastTargetErr[0] = rangeError;
        lastTargetErr[1] = headingError;
        lastTargetErr[2] = yawError;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        cmds[0] = Range.clip(cmds[0],  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
        cmds[1] = Range.clip(-cmds[1],  -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        cmds[2] = Range.clip(-cmds[2], -MAX_AUTO_TURN,   MAX_AUTO_TURN);

        return cmds;
    }

    /*************************************************************************/
    private void assignLocationVars(AprilTagDetection tag) {
        x = tag.ftcPose.x;
        y = tag.ftcPose.y;
        yaw = tag.ftcPose.yaw;
        range = tag.ftcPose.range;
        bearing = tag.ftcPose.bearing;
    }

}
