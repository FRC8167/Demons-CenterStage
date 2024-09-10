package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

public class DevilGripper {

    private Servo leftGripper, rightGripper;

    /** Servo Position Constants **/
    private final double OPEN_SETPNT = 0.3;
    private final double CLOSE_SETPT = 0.6;
//    private final double MID_SETPT = 0.6;

    public enum ToggleState {
        OPEN,
        CLOSED
    };

    ToggleState left_pos = ToggleState.OPEN;
    ToggleState right_pos = ToggleState.OPEN;

    /** /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\ **/
    public DevilGripper(Servo leftServo, Servo rightServo) {
        leftGripper = leftServo;
        rightGripper = rightServo;

        leftGripper.setDirection(Servo.Direction.REVERSE);
        rightGripper.setDirection(Servo.Direction.FORWARD);

        leftGripper.setPosition(CLOSE_SETPT);  //just a placeholder
        rightGripper.setPosition(CLOSE_SETPT);  //just a placeholder
    }

    /*************************************************************************/
    public void toggleLeftGripper() {

        switch (left_pos) {
            case OPEN:
                leftGripper.setPosition((CLOSE_SETPT));
                left_pos = ToggleState.CLOSED;
                break;
            case CLOSED:
                leftGripper.setPosition(OPEN_SETPNT);
                left_pos = ToggleState.OPEN;
                break;
        }
    }

    /*************************************************************************/
    public void toggleRightGripper() {

        switch (right_pos) {
            case OPEN:
                rightGripper.setPosition((CLOSE_SETPT));
                right_pos = ToggleState.CLOSED;
                break;
            case CLOSED:
                rightGripper.setPosition(OPEN_SETPNT);
                right_pos = ToggleState.OPEN;
                break;
        }
    }
}
