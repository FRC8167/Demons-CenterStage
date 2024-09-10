package org.firstinspires.ftc.teamcode.Utilities;

public class OmniDriveParams {

    public double drive, strafe, turn;

    public OmniDriveParams() {
        drive  = 0;
        strafe = 0;
        turn   = 0;
    }

    public void setDriveParams(double Drive, double Strafe, double Turn) {
        drive = Drive;
        strafe = Strafe;
        turn = Turn;
    }

    public void setDrive(double Drive) { drive = Drive; }
    public void setStrafe(double Strafe) { strafe = Strafe; }
    public void setTurn(double Turn) { turn = Turn; }


}
