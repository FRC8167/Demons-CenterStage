package org.firstinspires.ftc.teamcode;
import android.graphics.Camera;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Modules.ClimberSys;
import org.firstinspires.ftc.teamcode.Modules.DemonDrive;
import org.firstinspires.ftc.teamcode.Modules.DemonPincers;
import org.firstinspires.ftc.teamcode.Modules.DemonPivot;
import org.firstinspires.ftc.teamcode.Modules.DemonSlide;
import org.firstinspires.ftc.teamcode.Modules.DemonWrist;
import org.firstinspires.ftc.teamcode.Modules.DroneSys;
import org.firstinspires.ftc.teamcode.Modules.Vision;
import org.firstinspires.ftc.teamcode.Utilities.CameraIntrinsics;

import java.util.List;
import java.util.Locale;

public abstract class RobotBase_Demons extends LinearOpMode {

    public enum Alliance {
        RED,
        BLUE
    }

    CameraIntrinsics cam2Intrinsics = new CameraIntrinsics(782.7129, 782.7129, 279.9392,242.9770);

    static Alliance teamAlliance;

    /** Array to store control hubs **/
    private List<LynxModule> ctrlHubs;

    /******************* Globally Declared Sensors *******************/
    public IMU imu;
//    public DistanceSensor sensorDistance;
    public WebcamName webCam;

    /************ Define all Module Classes (SubSystems) *************/
    protected DemonDrive drive = null;
    protected DemonPincers pincers = null;
    protected DemonPivot pivotJoint = null;
    protected DemonSlide slideSys = null;
    protected DemonWrist wristJoint = null;
    protected DroneSys drone = null;
    protected ClimberSys climb = null;
    protected Vision vision = null;

    /*********************** Global Variables ***********************/
    public int exampleVariable = 0;

    /********************* Initialize Hardware *********************/
    public void initHardware() throws InterruptedException {

        /** Find all Control Hubs and Set Sensor Read Mode to AUTO **/
//        List<LynxModule> ctrlHubs;
        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /********************* Define Hardware Map Here *********************/
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        DcMotorEx armSlideMotor = hardwareMap.get(DcMotorEx.class, "armSlide");
        DcMotorEx armPivotMotor = hardwareMap.get(DcMotorEx.class, "armPivot");

        Servo leftPincer = hardwareMap.get(Servo.class, "leftPincer");
        Servo rightPincer = hardwareMap.get(Servo.class, "rightPincer");
        Servo droneServo = hardwareMap.get(Servo.class, "droneServo");
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo climberServo = hardwareMap.get(Servo.class, "climbServo");

//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        WebcamName webCam  = hardwareMap.get(WebcamName.class, "Webcam 2");

        /** Initialize/Assign Hardware to Module classes **/
        drive      = new DemonDrive(leftFront, leftRear, rightFront, rightRear);
        pincers    = new DemonPincers(leftPincer, rightPincer);
        pivotJoint = new DemonPivot(armPivotMotor);
        slideSys   = new DemonSlide(armSlideMotor);
        wristJoint = new DemonWrist(wristServo);
        drone      = new DroneSys(droneServo);
        climb      = new ClimberSys(climberServo);
        vision     = new Vision(webCam, cam2Intrinsics);
    }

    /************************ Accessor Functions **************************/
    public void setAlliance(Alliance allianceColor){
        teamAlliance = allianceColor;
    }

    public static Alliance getAlliance(){
        return teamAlliance;
    }

    /********************* Utility/Helper Functions ***********************/

    /* decPlace2
     * Return the string representation of a number to 2 decimal places
     * @param num decimal number input
     * @return string of number to 2 decimal places
     */
    public String decPlace2 (double num) {
        return String.format(Locale.getDefault(),"%.2f", num);
    }

    /*
     *  totalCurrentmA
     *
     * @return  Returns the total current draw from all control hubs in mA
     */
    public String totalCurrentmA() {
        double currentmA = 0;
        for (LynxModule hub : ctrlHubs) {
            currentmA += hub.getCurrent(CurrentUnit.MILLIAMPS);
        }
        return String.format(Locale.getDefault(), "%.3f mA", currentmA);
    }

    public String battVoltage() {
        double inVolts = 0;
        inVolts = ctrlHubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
        return String.format(Locale.getDefault(), "%.3f mA", inVolts);
    }

    public boolean debounce(boolean newVal, boolean oldVal ) {
        // Check for debounce
        if (newVal == oldVal) {
            return oldVal;               // Button State Has Not Changed
        } else {
            if (newVal) return true; else return false;
        }
    }


    /********************* runOpMOde must be Overridden in OpModes ***********************/
    /** Allows OpModes to connect to this base class **/
    @Override
    public abstract void runOpMode() throws InterruptedException;
}
