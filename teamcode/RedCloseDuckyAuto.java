package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name="RedCloseDuckyAuto", group = "AutoOp", preselectTeleOp = "atMainTeleOp")
//@Autonomous(name="RedCloseDuckyAuto")
public class RedCloseDuckyAuto extends LinearOpMode {
    String redDuckLocation = "";
    public Boolean redDuckFound = false;
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    public VisionPortal visionPortal;
    public ElapsedTime runtime = new ElapsedTime();


    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedKevinCube.tflite";

    private static final String[] LABELS = {
            "RD", };



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


    private boolean detectRedDuck() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3 && currentRecognitions.size() == 0) {
            sleep(250);
            currentRecognitions = tfod.getRecognitions();
        }
        if (currentRecognitions.size() > 0) {
            redDuckFound = true;
        } else {
            redDuckFound = false;
        }
        return redDuckFound;
    }

    private String detectRedDuckLocation() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft());
        telemetry.addData("# Object Location", currentRecognitions.size());
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3 && currentRecognitions.size() == 0) {
            sleep(250);
            currentRecognitions = tfod.getRecognitions();
        }
        if (x < 200) {
            redDuckLocation = "left";
        }
        if (x >= 200) {
            redDuckLocation = "center";
            };
        }
        return redDuckLocation;
    }

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        SampleMecanumDrive autoDrive = new SampleMecanumDrive(hardwareMap);
        robot.resetArmSystemEncoders();
        ElapsedTime runtime = new ElapsedTime();
        initTfod();

        robot.setAlliance(RobotHardware.AllianceColor.RED);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        Pose2d startLocation = new Pose2d(15.0, -65.0, Math.toRadians(90));

        TrajectorySequence forwardTraj = autoDrive.trajectorySequenceBuilder(startLocation)
                .forward(8)
                .turn(Math.toRadians(15))
                .build();

        TrajectorySequence scoreFromLeftSpike = autoDrive.trajectorySequenceBuilder(startLocation)
                //RUNS IF Red DUCK FOUND ON LEFT SPIKE
                .forward(16.5)
                .turn(Math.toRadians(34))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.openRightPincer())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(1.5)
                .back(5)
                .strafeRight(20)
                .lineToSplineHeading(new Pose2d(59, -16, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(3.6)
                .back(2)
                .strafeRight(47)
                .forward(10)
                .build();

        TrajectorySequence moveToCenterTraj = autoDrive.trajectorySequenceBuilder(startLocation)
                //RUNS IF DUCK NOT FOUND ON LEFT SPIKE
                .forward(8)
                .build();

        TrajectorySequence scoreFromCenterSpike = autoDrive.trajectorySequenceBuilder(moveToCenterTraj.end())
                //RUNS IF Red DUCKY FOUND ON CENTER SPIKE
                .forward(17.5)
                .turn(Math.toRadians(-25))
//                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(2.0)
                .back(5)
                .strafeRight(5)
                .lineToSplineHeading(new Pose2d(58, -33, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(4.5)
                .back(2)
                .strafeRight(35)
                .forward(10)
                .build();


        TrajectorySequence scoreFromRightSpike = autoDrive.trajectorySequenceBuilder(moveToCenterTraj.end())
                //RUNS IF NO DUCKY FOUND ON LEFT OR CENTER SPIKE
                .forward(16)
                .turn(Math.toRadians(-80))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.openRightPincer())
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(1.5)
                .turn(Math.toRadians(-10))
                .strafeRight(25)
                .lineToSplineHeading(new Pose2d(56, -41, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(4)
                .back(2)
                .strafeRight(24)
                .forward(10)
                .build();


        waitForStart();
        autoDrive.setPoseEstimate(startLocation);

        if (detectRedDuck() && detectRedDuckLocation() == "left") {
//            redDuckLocation = "left";
            telemetry.addData("Red Duck Found", redDuckFound);
            telemetry.addData("Location", redDuckLocation);
            telemetryTfod();
            telemetry.update();
            autoDrive.followTrajectorySequence(scoreFromLeftSpike);
//            requestOpModeStop();
        } else {
            autoDrive.followTrajectorySequence(moveToCenterTraj);
            sleep(1);
            if (detectRedDuck() && detectRedDuckLocation() == "center") {
//                redDuckLocation = "center";
                telemetry.addData("Red Duck Found", redDuckFound);
                telemetry.addData("Location", redDuckLocation);
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




