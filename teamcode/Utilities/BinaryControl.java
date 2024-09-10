package org.firstinspires.ftc.teamcode.Utilities;

public class BinaryControl {

    private boolean currentState;
    private boolean lastState;
    private boolean controlState;

    /**
     * /////////////////////////// CONSTRUCTOR \\\\\\\\\\\\\\\\\\\\\\\\\\\
     *
     * @param
     **/
    public BinaryControl(boolean initialValue) {
        currentState = initialValue;
        lastState = currentState;
    }

    public void update(boolean buttonValue) {

        lastState = currentState;
        currentState = buttonValue;

        if (currentState && !lastState) {
            controlState = true;
        } else controlState = false;
    }

    public boolean state() {
        return controlState;
    }
}
