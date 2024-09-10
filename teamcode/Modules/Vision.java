package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotBase_Demons;
import org.firstinspires.ftc.teamcode.Utilities.CameraIntrinsics;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {

    public final int BLUE_LEFT = 1;
    public final int BLUE_CENTER = 2;
    public final int BLUE_RIGHT = 3;
    public final int RED_LEFT = 4;
    public final int RED_CENTER = 5;
    public final int RED_RIGHT = 6;

    /** WebCam1 Camera Settings **/
    private long CAMERA_EXPOSURE = 6; //15;
    private int  CAMERA_GAIN = 250;
//    private double LENS_INTRINSICS_FX = 889.035;
//    private double LENS_INTRINSICS_FY = 889.035;
//    private double LENS_INTRINSICS_CX = 390.3019;
//    private double LENS_INTRINSICS_CY = 66.3539;


    /** WebCam2 Camera Settings **/
//    private long CAMERA_EXPOSURE = 6; //15;
//    private int  CAMERA_GAIN = 250;
//    private double LENS_INTRINSICS_FX = 782.7129;
//    private double LENS_INTRINSICS_FY = 782.7129;
//    private double LENS_INTRINSICS_CX = 279.9392;
//    private double LENS_INTRINSICS_CY = 242.9770;

    /** The variable to hold our camera object **/
    private WebcamName cam = null;

    /** The variable to store our instance of the AprilTag processor. **/
    private AprilTagProcessor aprilTag = null;

    /** The variable to store our instance of the TensorFlow Object Detection processor **/
    private TfodProcessor tfod = null;
    private static final String[] BLUE_LABELS = { "BD", };
    private static final String[] RED_LABELS = { "RD", };
    private String[] labels = {""};

    /* TFOD model files */
    private static final String blueTeamPropTfodFile = "BlueDucky.tflite";
    private static final String redTeamPropTfodFile = "RedDucky.tflite";

    /* TFOD Minimum Confidence Level for Detections */
    private float confidence = 0.75f;

    public Boolean teamPropFound = false;

    /** The variable to store our instance of the vision portal **/
    private VisionPortal visionPortal = null;

    public ElapsedTime runtime = new ElapsedTime();

    private List<AprilTagDetection> currentDetections = null;


    /** /////////////////////////// CONSTRUCTORS \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public Vision(WebcamName camera) throws InterruptedException {
        cam = camera;
        buildAprilTagProcessor();
        buildTfodProcessor();
        buildVisionPortal();
    }

    public Vision(WebcamName Camera, CameraIntrinsics intrinsics) throws InterruptedException {
        cam = Camera;
        buildAprilTagProcessor(intrinsics);
        buildTfodProcessor();
        buildVisionPortal();
    }

    /*************************************************************************/
    private void buildAprilTagProcessor() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(2);
    }

    /*************************************************************************/
    private void buildAprilTagProcessor(CameraIntrinsics intrinsics) {
        aprilTag = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .setLensIntrinsics(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(2);
    }

    /*************************************************************************/
    private void buildTfodProcessor() {

        String TFOD_MODEL_ASSET;
        if (RobotBase_Demons.getAlliance() == RobotBase_Demons.Alliance.BLUE) {
            TFOD_MODEL_ASSET = blueTeamPropTfodFile;
            labels = BLUE_LABELS;
        } else {
            TFOD_MODEL_ASSET = redTeamPropTfodFile;
            labels = RED_LABELS;
        }

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(labels)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)                            // Could this be the ducky problem
                .build();

        setTfodConfidenceLevel(confidence);
    }

    /*************************************************************************/
    private void buildVisionPortal() throws InterruptedException {

        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessors(tfod, aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        /** Pause code execution until camera state is streaming **/
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }
        setManualExposure(CAMERA_EXPOSURE, CAMERA_GAIN);

        /** Initialize to Disable to conserve processing power **/
        disableTfodDetection();
        disableAprilTagDetection();
    }

    /*************************************************************************/
    /*
        Manually set the camera gain and exposure.
        This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(long exposureMS, int cameraGain) {

        if (visionPortal == null) {
            return;
        }

        /** Adjust exposure and gain settings to reduce motion blur **/
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure.isExposureSupported()) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(cameraGain);
        }
    }

    /*************************** Processor Controls ***************************/
    public void enableAprilTagDetection() {
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableAprilTagDetection() {
        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    public void enableTfodDetection() {
        visionPortal.setProcessorEnabled(tfod, true);
    }
    public void disableTfodDetection() {
        visionPortal.setProcessorEnabled(tfod, false);
    }


    /* *==============================================================================* */
    /*                               April Tag FUnctions                                */
    /* *==============================================================================* */

    /** Must be called prior to any other April Tag Functions **/
    public void scanForAprilTags() { currentDetections = aprilTag.getDetections(); }

    /*************************************************************************/
//    public List<AprilTagDetection> allAprilTagsDetected() { return aprilTag.getDetections(); }
    public List<AprilTagDetection> allAprilTagsDetected() { return currentDetections; }

    /*************************************************************************/
    public boolean aprilTagsDetected() {
//        currentDetections = aprilTag.getDetections();
        if(currentDetections.size() > 0) return true; else return false;
    }

    /*************************************************************************/
    public boolean aprilTagAvailable(int tagId) {;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                return true;
            }
        }
        return false;
    }

    /*************************************************************************/
    public int aprilTagsDetectedCount() { return currentDetections.size(); }

    /*************************************************************************/
    public AprilTagDetection getTagData(int tagId) {
        AprilTagDetection tag = null;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                tag = detection;
                break;  // don't look any further.
            }
        }
        return tag;
    }


    /* *==============================================================================* */
    /*                                  TFOD FUnctions                                  */
    /* *==============================================================================* */

    public boolean detectTeamProp() {
        List<Recognition> currentTfodRecognitions = tfod.getRecognitions();
        if (currentTfodRecognitions.size() > 0) teamPropFound = true; else teamPropFound = false;
        return teamPropFound;
    }

    /*************************************************************************/
    public String detectTeamPropLocation() throws InterruptedException {
        String propLocation = "";
        List<Recognition> currentTfodRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentTfodRecognitions) {
            double x = (recognition.getLeft());
            runtime.reset();;
            while (runtime.seconds() < 1 && currentTfodRecognitions.size() == 0) {
                sleep(250);
                currentTfodRecognitions = tfod.getRecognitions();
            }
            if (x < 200) {
                propLocation = "left";
            }
            if (x >= 200) {
                propLocation = "center";
            };
        }
        return propLocation;
    }

    /*************************************************************************/
    public List<Recognition> tfodDetections() {
        return tfod.getRecognitions();
    }

    /*************************************************************************/
    public void setTfodConfidenceLevel(float level) {
        tfod.setMinResultConfidence(level);
    }
}
