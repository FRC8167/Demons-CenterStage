package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

public class DemonWrist {

    Servo wristServo;

    public final double POS_NEST = .95;
    public final double POS_FLOOR_PICKUP = 0.8628;  //.45;
    public final double POS_SCORE_LOW = 0.8744;     //.4;
    public final double POS_SCORE_HIGH = 0.8588;    //.4;
    public final double SERVO_FLOOR_DROP = .8744;
//    public final double POS_TRAVEL = 0.1;

    private final double ROTATE_LIMIT_LOW = 0.8500; //0.4;
    private final double ROTATE_LIMIT_HIGH = .96;

    /**
     * /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\
     **/
    public DemonWrist(Servo Servo) {
        wristServo = Servo;
    }

    /*************************************************************************/
    public double clampSetpt(double setPoint) {
        if (setPoint > ROTATE_LIMIT_HIGH) {
            setPoint = ROTATE_LIMIT_HIGH;
        } else if (setPoint < ROTATE_LIMIT_LOW) {
            setPoint = ROTATE_LIMIT_LOW;
        }
        return setPoint;
    }

    /*************************************************************************/
    public void setPosition(double setPoint) {
        wristServo.setPosition(clampSetpt(setPoint));
    }

    /*************************************************************************/
    public void update() {
        double currentPos = getServoPos();
        if (currentPos > ROTATE_LIMIT_HIGH || currentPos < ROTATE_LIMIT_LOW) {
            wristServo.setPosition(clampSetpt(currentPos));
        }
    }

    /** Accessors **/
    public double getServoPos() { return wristServo.getPosition(); }

}

