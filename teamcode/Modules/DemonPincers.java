package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

public class DemonPincers {

    private final Servo leftPincer, rightPincer;

    /** Servo Position Constants **/
    private final double OPEN_SETPT = 0.35;
    private final double CLOSE_SETPT = 0.6;
//    private final double MID_SETPT = 0.6;

    public enum PincerState {
        OPEN,
        CLOSED
    };

    PincerState left_pos = PincerState.OPEN;
    PincerState right_pos = PincerState.OPEN;

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DemonPincers(Servo leftServo, Servo rightServo) {

        leftPincer = leftServo;
        rightPincer = rightServo;

        leftPincer.setDirection(Servo.Direction.REVERSE);
        rightPincer.setDirection(Servo.Direction.FORWARD);

        leftPincer.setPosition(CLOSE_SETPT);  //just a placeholder
        rightPincer.setPosition(CLOSE_SETPT);  //just a placeholder
    }

    /*************************************************************************/
    public void toggleLeftPincer() {

        switch (left_pos) {
            case OPEN:
                leftPincer.setPosition((CLOSE_SETPT));
                left_pos = PincerState.CLOSED;
                break;
            case CLOSED:
                leftPincer.setPosition(OPEN_SETPT);
                left_pos = PincerState.OPEN;
                break;
        }
    }

    /*************************************************************************/
    public void toggleRightPincer() {

        switch (right_pos) {
            case OPEN:
                rightPincer.setPosition((CLOSE_SETPT));
                right_pos = PincerState.CLOSED;
                break;
            case CLOSED:
                rightPincer.setPosition(OPEN_SETPT);
                right_pos = PincerState.OPEN;
                break;
        }
    }

    /*************************************************************************/
    public void openPincers() {

        leftPincer.setPosition(OPEN_SETPT);
        rightPincer.setPosition(OPEN_SETPT);

        left_pos = PincerState.OPEN;
        right_pos = PincerState.OPEN;
    }

    /*************************************************************************/
    public void closePincers() {

        leftPincer.setPosition(CLOSE_SETPT);
        rightPincer.setPosition(CLOSE_SETPT);

        left_pos = PincerState.CLOSED;
        right_pos = PincerState.CLOSED;
    }

}
