package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.Locale;

public abstract class RobotBase extends LinearOpMode {

    /******************* Globally Declared Sensors *******************/
    public IMU imu; // Currently not used

    /************ Define all Module Classes (SubSystems) *************/
    //public Arm arm = null;
    public MOD_Drive drive = null;

    /*********************** Global Variables ***********************/
    public int exampleVariable = 0;

    /********************* Initialize Hardware *********************/
    public void initHardware() throws InterruptedException {

        /** Find all Control Hubs and Set Sensor Read Mode to AUTO **/
        List<LynxModule> ctrlHubs;
        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /********************* Define Hardware Map Here *********************/

        // Drive Motors
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Arm Motors
        DcMotor lowerPivot = hardwareMap.get(DcMotor.class, "lowerPivot");
        DcMotor upperPivot = hardwareMap.get(DcMotor.class, "upperPivot");

        // Grabber Servos
        Servo leftGripper  = hardwareMap.get(Servo.class, "leftGripper");
        Servo rightGripper = hardwareMap.get(Servo.class, "rightGripper");

        // Wrist Servo
        Servo armWrist     = hardwareMap.get(Servo.class, "armWrist");

        // Drone Launch Servo
        Servo droneServo   = hardwareMap.get(Servo.class, "droneServo");

        /** Initialize/Assign Module classes to Hardware **/
        drive = new MOD_Drive(leftFront, leftRear, rightFront, rightRear);

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


    /********************* runOpMOde must be Overridden in OpModes ***********************/
    /** Allows OpModes to connect to this base class **/
    @Override
    public abstract void runOpMode() throws InterruptedException;
}
