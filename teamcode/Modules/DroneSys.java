package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneSys {

    private Servo droneSrvo;

    private boolean launcherReady;

    /** Servo Position Constants **/
    private final double STOWE = 1.0;
    private final double LAUNCH = 0.60;

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DroneSys(Servo dronetServo) {
        droneSrvo = dronetServo;
        launcherReady = true;
    }

    /*************************************************************************/
    public void launchDrone(){
        droneSrvo.setPosition(LAUNCH);
        launcherReady = false;
    }

    /*************************************************************************/
    public void resetDroneLauncher(){
        droneSrvo.setPosition(STOWE);
        launcherReady = false;
    }

}
