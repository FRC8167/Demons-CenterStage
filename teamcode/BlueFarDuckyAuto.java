package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="BlueFarDuckyAuto", group = "AutoOp", preselectTeleOp = "atMainTeleOp")
//@Autonomous(name="BlueFarDuckyAuto")
public class BlueFarDuckyAuto extends LinearOpMode {
    String blueDuckLocation = "";
    public Boolean blueDuckFound = false;
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    public VisionPortal visionPortal;
    public ElapsedTime runtime = new ElapsedTime();


    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "BlueKevinCube.tflite";

    private static final String[] LABELS = {
            "BlueKevin", };

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.75f);
        visionPortal.setProcessorEnabled(tfod, true);
    }


    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()


    private boolean detectBlueDuck() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3 && currentRecognitions.size() == 0) {
            sleep(250);
            currentRecognitions = tfod.getRecognitions();
        }
        if (currentRecognitions.size() > 0) {
            blueDuckFound = true;
        } else {
            blueDuckFound = false;
        }
        return blueDuckFound;
    }

    private String detectBlueDuckLocation() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        runtime.reset();
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft());
            telemetry.addData("# Object Location", currentRecognitions.size());
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1 && currentRecognitions.size() == 0) {
                sleep(250);
                currentRecognitions = tfod.getRecognitions();
            }
            if (x < 200) {
                blueDuckLocation = "left";
            }
            if (x >= 200) {
                blueDuckLocation = "center";
            };
        }
        return blueDuckLocation;
    }



    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        SampleMecanumDrive autoDrive = new SampleMecanumDrive(hardwareMap);
        robot.resetArmSystemEncoders();
        ElapsedTime runtime = new ElapsedTime();
        initTfod();

        robot.setAlliance(RobotHardware.AllianceColor.BLUE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        Pose2d startLocation = new Pose2d(-40, 65.0, Math.toRadians(-90));

        TrajectorySequence forwardTraj = autoDrive.trajectorySequenceBuilder(startLocation)
                .forward(8)
                .turn(Math.toRadians(15))
                .build();

        TrajectorySequence scoreFromLeftSpike = autoDrive.trajectorySequenceBuilder(startLocation)
                //RUNS IF BLUE DUCK FOUND ON LEFT SPIKE
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .waitSeconds(0.5)
                .forward(18)
                .turn(Math.toRadians(35))
//                .forward(11)
//                .back(5.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.openRightPincer())
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(0.75)
                .strafeRight(24)
                .lineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(0)))  //-15
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(34, 50), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(3)
                .back(2)
                .strafeRight(44)
                .forward(10)
                .build();

        TrajectorySequence moveToCenterTraj = autoDrive.trajectorySequenceBuilder(startLocation)
                //RUNS IF DUCK NOT FOUND ON LEFT SPIKE
                .forward(8)
                .build();

        TrajectorySequence scoreFromCenterSpike = autoDrive.trajectorySequenceBuilder(moveToCenterTraj.end())
                //RUNS IF BLUE DUCKY FOUND ON CENTER SPIKE
                .forward(19)
                .turn(Math.toRadians(-25))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.openRightPincer())
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_TRAVEL-400, 1))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(1.5)
//                .back(8)
                .strafeRight(28)
                .lineToLinearHeading(new Pose2d(-68, 6, Math.toRadians(0)))  //-15
                .lineToLinearHeading(new Pose2d(20, 6, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(33, 48), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-3, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(3)
                .strafeRight(34)
                .back(2)
                .forward(10)
                .build();


        TrajectorySequence scoreFromRightSpike = autoDrive.trajectorySequenceBuilder(moveToCenterTraj.end())
                //RUNS IF NO DUCKY FOUND ON LEFT OR CENTER SPIKE
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .waitSeconds(0.5)
                .turn(Math.toRadians(-65))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.openRightPincer())
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.pivotMoveToCountPosition((robot.PIVOT_POS_TRAVEL-400), 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(1.0)
                .turn(Math.toRadians(70))
//                .lineToLinearHeading(new Pose2d(-44, 15, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-44, 4, Math.toRadians(0)))  //-15
                .lineToLinearHeading(new Pose2d(20, 4, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(38, 40), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-3, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_SCORE_LOW-200, 1))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_SCORE_LOW-100, 1))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.wristServo.setPosition(robot.SERVO_SCORE_LOW))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(3)
                .back(2)
                .strafeRight(30)
                .forward(10)
                .build();


        waitForStart();
        autoDrive.setPoseEstimate(startLocation);

        if (detectBlueDuck() && detectBlueDuckLocation() == "left") {
            telemetry.addData("Blue Duck Found", blueDuckFound);
            telemetry.addData("Location", blueDuckLocation);
            telemetryTfod();
            telemetry.update();
            autoDrive.followTrajectorySequence(scoreFromLeftSpike);
//            requestOpModeStop();
        } else {
            autoDrive.followTrajectorySequence(moveToCenterTraj);
            if (detectBlueDuck()  && detectBlueDuckLocation() == "center") {
                telemetry.addData("Blue Duck Found", blueDuckFound);
                telemetry.addData("Location", blueDuckLocation);
                telemetryTfod();
                telemetry.update();
                autoDrive.followTrajectorySequence(scoreFromCenterSpike);
//                sleep(10000);
//                requestOpModeStop();
            } else {
                autoDrive.followTrajectorySequence(scoreFromRightSpike);
//                requestOpModeStop();
            }
        }

    }
}




