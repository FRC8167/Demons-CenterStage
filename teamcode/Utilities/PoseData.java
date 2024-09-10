package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class PoseData {

    private double x, y, theta;

    public PoseData()
    {
        x = 0;
        y = 0;
        theta = 0;
    }

    public void setPose(AprilTagDetection tag) {
        x = tag.ftcPose.x;
        y = tag.ftcPose.y;
        theta = tag.ftcPose.yaw;
    }

    public void setPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double x()     { return x; }
    public double y()     { return y; }
    public double theta() { return theta; }

    public void x(double value) {
        x = value;
    }
    public void y(double value) {
        y = value;
    }
    public void theta(double value) {
        theta = value;
    }
}
