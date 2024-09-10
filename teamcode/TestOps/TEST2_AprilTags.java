package org.firstinspires.ftc.teamcode.TestOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BinaryControl;
import org.firstinspires.ftc.teamcode.RobotBase_Demons;
import org.firstinspires.ftc.teamcode.Utilities.AprilTagApproach;
import org.firstinspires.ftc.teamcode.Utilities.PoseData;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Disabled
@TeleOp(name="TestAprilTags", group="Test")
public class TEST2_AprilTags extends RobotBase_Demons {

    /** Rates **/
    public static double acqHz  = 40;    // 200Hz sample rate ~25ms to process an image;
    public static double ctrlHz = 20;    // 10Hz control rate (update controls 10 times per sec)
    public static double filterCutoffHz = 10.0;
    public static double xOverDistance  = 100;

    public static double[] driveCmds = {0,0,0};

    public static double RANGE_OFFSET   = 16;
    public static double LATERAL_OFFSET = -1;
    public static double YAW_OFFSET     = 0;

    public static double KP_DRIVE  = 0.025; //0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double KP_STRAFE = 0.070; //0.15;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double KP_YAW    = 0.008; //0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double KD_DRIVE  = 0.001;
    public static double KD_STRAFE = 0.001;
    public static double KD_YAW    = 0.001;

    private static double MAX_AUTO_SPEED  = 0.35;   //  Clip the approach speed to this max value (adjust for your robot)
    private static double MAX_AUTO_STRAFE = 0.35;   //  Clip the approach speed to this max value (adjust for your robot)
    private static double MAX_AUTO_TURN   = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    private int tagsInView = 0;
    private double x, y, yaw, range, bearing;
    private double[] lastTargetErr = {0, 0, 0};
    private double deltaT = 1;
    private List<AprilTagDetection> tagsIdentified;




    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(); // Calls super initHardware (from RobotBase_Demons)

        ElapsedTime acqTimer  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime ctrlTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        BinaryControl driveBtn = new BinaryControl(gamepad1.right_bumper);

        AprilTagApproach aprilTag = new AprilTagApproach(ctrlHz);

        // FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** Initialization should take place in an auto and not overwritten here **/
        /** This is for teleOp evaluation/debug use only                         **/
//
        setAlliance(Alliance.BLUE);
        /**----------------------------------------------------------------------**/

//        vision.enableAprilTagDetection();
        deltaT = 1/ctrlHz;

        waitForStart();

        /* ..................................................................... */
        while (opModeIsActive()) {
            deltaT = 1/ctrlHz;

            driveBtn.update(gamepad1.right_bumper);

            /** Acquire Sensor Data **/
//            if(acqTimer.milliseconds() > 1/acqHz) {
                acqTimer.reset();
                vision.scanForAprilTags();
                tagsInView = vision.aprilTagsDetectedCount();
                tagsIdentified = vision.allAprilTagsDetected();
//            }

            /** Tags available and driver wants to shoot and automatic approach **/
            if (ctrlTimer.milliseconds() > 1/ctrlHz) {
                ctrlTimer.reset();
                if(gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                    if (vision.aprilTagAvailable(vision.BLUE_LEFT) && (gamepad1.dpad_left)) {
                        driveCmds = shootPidApproach(vision.getTagData(vision.BLUE_LEFT));
                        assignLocationVars(vision.getTagData(vision.BLUE_LEFT));
                    }
//                    if (vision.aprilTagAvailable(vision.BLUE_CENTER) && (gamepad1.dpad_up)) {
//                        driveCmds = shootPidApproach(vision.getTagData(vision.BLUE_CENTER));
//                    }
//                    if (vision.aprilTagAvailable(vision.BLUE_RIGHT) && (gamepad1.dpad_right)) {
//                        driveCmds = shootPidApproach(vision.getTagData(vision.BLUE_RIGHT));
//                    }

                    drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);    // Full motion
                } else {
                    driveCmds[0] = -gamepad1.left_stick_y;
                    driveCmds[1] =  gamepad1.left_stick_x;
                    driveCmds[2] =  gamepad1.right_stick_x;
                    drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);
                }

                /***************** Uncomment when ready to change axis *****************/
//                if(driveBtn.state()) {
//                    drive.mecanumDrive(driveCmds[0], 0, 0);        // FWD/AFT motion only
//                    drive.mecanumDrive(0, driveCmds[1], 0);                 // Strafe motion only
//                    drive.mecanumDrive(driveCmds[0], 0, driveCmds[2]);                 // Turn motion only
//                     drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);    // Full motion
//                }
                /**********************************************************************/
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
//            telemetry.addData("Drive Y,X,Yaw: ", " %5.1f%, %5.1f%, %5.1f%", driveCmds[0]*100, driveCmds[1]*100, driveCmds[2]*100);
            telemetry.addData("Drive", " %5.1f",drvcmd);
            telemetry.addData("Strafe", " %5.1f",strcmd);
            telemetry.addData("Turn", " %5.1f",thecmd);

            telemetry.update();
        }
    }

    /*************************************************************************/
    public double[] shootPidApproach(AprilTagDetection targetTag) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = targetTag.ftcPose.range - RANGE_OFFSET;
//        double  headingError = targetTag.ftcPose.y + HEADING_OFFSET;
//        double  yawError     = targetTag.ftcPose.yaw + YAW_OFFSET;

        double crossOverDistance = xOverDistance;

        driveCmds = nearApproach(targetTag);

//        if (rangeError > crossOverDistance) {
//            /** Use Range, Heading and Yaw for rough navigation to tag **/
//            driveCmds = farApproach(targetTag);
//        } else {
//            /** Use X, Y and Yaw for precise moves near target tag **/
//            driveCmds = nearApproach(targetTag);
//        }

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        driveCmds[0] = Range.clip(driveCmds[0], -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
        driveCmds[1] = Range.clip(driveCmds[1], -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        driveCmds[2] = Range.clip(-driveCmds[2], -MAX_AUTO_TURN,   MAX_AUTO_TURN);

        return driveCmds;
    }

    /*************************************************************************/
    private double[] farApproach(AprilTagDetection tag) {

        double rangeError   = tag.ftcPose.range   - RANGE_OFFSET;
        double headingError = tag.ftcPose.bearing - LATERAL_OFFSET;
        double yawError     = tag.ftcPose.yaw     - YAW_OFFSET;

        double[] cmds = {0.0, 0.0, 0.0};

        // Calculate PD Commands
        cmds[0] = KP_DRIVE  * rangeError   + KD_DRIVE  * (rangeError   - lastTargetErr[0]) / deltaT;
        cmds[1] = KP_STRAFE * yawError     + KD_STRAFE * (headingError - lastTargetErr[1]) / deltaT;
        cmds[2] = KP_YAW    * headingError + KD_YAW    * (yawError     - lastTargetErr[2]) / deltaT;

        lastTargetErr[0] = rangeError;
        lastTargetErr[1] = headingError;
        lastTargetErr[2] = yawError;

        return cmds;
    }

    /*************************************************************************/
    private double[] nearApproach(AprilTagDetection tag) {

        double yError   = tag.ftcPose.y   - RANGE_OFFSET;
        double xError   = tag.ftcPose.x   - LATERAL_OFFSET;
        double yawError = tag.ftcPose.yaw - YAW_OFFSET;

        double[] cmds = {0, 0, 0};

        // Calculate PD Commands
        cmds[0] = KP_DRIVE * yError + KD_DRIVE * (yError - lastTargetErr[0]) / deltaT;

//        if(xError < -0.25 || xError > 0.25) {
        cmds[1] = KP_STRAFE * xError + KD_STRAFE * (xError - lastTargetErr[1]) / deltaT;
//        } else cmds[1] = 0;

        cmds[2] = KP_YAW * yawError * Math.cos(Math.toRadians(yError)) + KD_YAW * (yawError - lastTargetErr[2]) / deltaT;

        lastTargetErr[0] = yError;
        lastTargetErr[1] = xError;
        lastTargetErr[2] = yawError;

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
