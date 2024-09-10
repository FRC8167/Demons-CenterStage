package org.firstinspires.ftc.teamcode.TestOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Vision;
import org.firstinspires.ftc.teamcode.RobotBase_Demons;
import org.firstinspires.ftc.teamcode.Utilities.AprilTagApproach;
import org.firstinspires.ftc.teamcode.Utilities.LowPassFilter;
import org.firstinspires.ftc.teamcode.Utilities.PIDcontroller;
import org.firstinspires.ftc.teamcode.Utilities.PoseData;

//@Config
@Disabled
@TeleOp(name="NoTestAprilTags", group="Test")
public class TEST_AprilTags extends RobotBase_Demons {

    /** Rates **/
    public static double acqHz = 30;     // 200Hz sample rate ~60ms to process an image;
    double ctrlHz = 20;                  // 10Hz control rate (update controls 10 times per sec)

    public static double filterCutoffHz = 10.0;

    public static double[] driveCmds = {0.0,0.0,0.0};

    /** Sensor Filters **/
//    LowPassFilter xSignal     = new LowPassFilter(filterCutoffHz, 1/acqHz);
//    LowPassFilter ySignal     = new LowPassFilter(filterCutoffHz, 1/acqHz);
//    LowPassFilter thetaSignal = new LowPassFilter(filterCutoffHz, 1/acqHz);

    /**  Pose Defines **/
    PoseData rawPose       = new PoseData();
//    PoseData filteredPose  = new PoseData();
//    PoseData driveCommands = new PoseData();

    /** Axis PID Controllers **/
//    static double kpx     = 0.025;
//    static double kpy     = 0.025;
//    static double kptheta = 0.010;
//    static double kdx     = 0.0;
//    static double kdy     = 0.0;
//    static double kdtheta = 0.0;

//    PIDcontroller xAxisCmd = new PIDcontroller(kpx , kdx, ctrlHz);
//    PIDcontroller yAxisCmd = new PIDcontroller(kpy , kdy, ctrlHz);
//    PIDcontroller thetaCmd = new PIDcontroller(kptheta , kdtheta, ctrlHz);

    private int tagsInView = 0;
//    AprilTagApproach autoApproach = new AprilTagApproach();


    @Override
    public void runOpMode() throws InterruptedException {

        initHardware(); // Calls super initHardware (from RobotBase_Demons)

        ElapsedTime acqTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime ctrlTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        AprilTagApproach aprilTag = new AprilTagApproach(ctrlHz);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** Initialization should take place in an auto and not overwritten here **/
        /** This is for teleOp evaluation/debug use only                         **/
//        pivotJoint.resetEncoder();
//        slideSys.resetEncoder();
//
//        setAlliance(AllianceColor.BLUE);
        /**----------------------------------------------------------------------**/
//        Vision vision = new Vision(RobotBase_Demons.webCam);
        vision.enableAprilTagDetection();

        waitForStart();

        /* ..................................................................... */
        while (opModeIsActive()) {

            /** Acquire Sensor Data **/
            if(acqTimer.milliseconds() > 1/acqHz) {
                acqTimer.reset();
                vision.scanForAprilTags();
                tagsInView =  vision.aprilTagsDetectedCount();
            }

            /** Tags available and driver wants to shoot and automatic approach **/
            if (ctrlTimer.milliseconds() > 1/ctrlHz) {
                ctrlTimer.reset();
                if (vision.aprilTagAvailable(vision.BLUE_LEFT) && (gamepad1.dpad_left)) {
                    rawPose.setPose(vision.getTagData(vision.BLUE_LEFT));
                    driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_LEFT));
//                }
//                if (vision.aprilTagAvailable(vision.BLUE_CENTER) && (gamepad1.dpad_up)) {
//                    rawPose.setPose(vision.getTagData(vision.BLUE_CENTER));
//                    driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_CENTER));
//                }
//                if (vision.aprilTagAvailable(vision.BLUE_RIGHT) && (gamepad1.dpad_right)) {
//                    rawPose.setPose(vision.getTagData(vision.BLUE_RIGHT));
//                    driveCmds = aprilTag.shootPidApproach(vision.getTagData(vision.BLUE_RIGHT));
                } else {
                    driveCmds[0] = -gamepad1.left_stick_y * .6;
                    driveCmds[1] =  gamepad1.left_stick_x * .6;
                    driveCmds[2] =  gamepad1.right_stick_x * .6;
                }

                /******************** Uncomment when ready to move ********************/
                drive.mecanumDrive(driveCmds[0], driveCmds[1], driveCmds[2]);
                /**********************************************************************/
            }

            telemetry.addData("Tags in View:", tagsInView);
            telemetry.addData("Filter Hz: ", filterCutoffHz);

            telemetry.addData("\nTag Y X Yaw", "%5.2f, %5.2f, %5.2f", rawPose.y(), rawPose.x(), rawPose.theta());
//            telemetry.addData("y Axis:","Raw %5.2f, Flt %5.2f", rawPose.y(), filteredPose.y());
//            telemetry.addData("x Axis:","Raw %5.2f, Flt %5.2f", rawPose.x(), filteredPose.x());
//            telemetry.addData("Theta: ","Raw %5.2f, Flt %5.2f", rawPose.theta(), filteredPose.theta());
//
//            telemetry.addData("X PID: ", "kp %3,3f, kd %3.3f", xAxisCmd.GAIN.p,xAxisCmd.GAIN.d);
//            telemetry.addData("Y PID: ", "kp %3,3f, kd %3.3f", yAxisCmd.GAIN.p, yAxisCmd.GAIN.d); //    kpy, kdy);
//            telemetry.addData("Theta PID: ", "kp %3,3f, kd %3.3f", kptheta, kdtheta);

            telemetry.addData("Driue Y,X,Yaw: ", " %5.2f, %5.2f, %5.2f", driveCmds[0], driveCmds[1], driveCmds[2]);

            telemetry.update();

        }
    }
}
