package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClimberSys {

    private Servo climbServo;

    public double STOW_POSITION = 0.425;

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public ClimberSys(Servo servo) {
        climbServo = servo;
    }

    /*************************************************************************/
    public void manualPosition(double position) {

    }

    /*************************************************************************/
    public void diable() {
        ((ServoImplEx)climbServo).setPwmDisable();
    }

    /*************************************************************************/
    public void enable() {
        ((ServoImplEx)climbServo).setPwmEnable();
    }

}
