package org.firstinspires.ftc.teamcode.Utilities;

public class CameraIntrinsics {

    public long exposure = 6;
    public int  gain = 250;

    public double fx = 0;
    public double fy = 0;
    public double cx = 0;
    public double cy = 0;

    public CameraIntrinsics(double Fx, double Fy, double Cx, double Cy) {
        fx = Fx;
        fy = Fy;
        cx = Cx;
        cy = Cy;
    }

}
