package org.firstinspires.ftc.teamcode.TestOps;

import static java.lang.Math.cos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.DemonPivot;
import org.firstinspires.ftc.teamcode.RobotBase_Demons;
import org.firstinspires.ftc.teamcode.Utilities.AprilTagApproach;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@TeleOp(name="testOpArmFF", group="Test")
public class testOp_ArmFeedForward extends RobotBase_Demons {

    public static double kf;
    public static double currentAngle;
//    public static double ticksPerDeg = 537.1 * (100/20) * (1/(2*3.14159)) * (3.14159/180);
    public static double degPerTick = 1/7125.5 * 360;

    @Override
    public void runOpMode() throws InterruptedException {

        kf = 0;

        initHardware(); // Calls super initHardware (from RobotBase_Demons)
        pivotJoint.resetEncoder();
        slideSys.resetEncoder();

        currentAngle = pivotJoint.getPivotPos() / degPerTick - 50;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        /** ................................................................... **/
        while (opModeIsActive()) {

            /** Calculate Feed Forward term **/
            currentAngle = pivotJoint.getPivotPos() * degPerTick -50;
            kf = 0.0075 * slideSys.getSlidePos() * cos(Math.toRadians(currentAngle)) + 0.28;
            pivotJoint.setFeedForwardTerm(kf);


            /** Manual movement of Pivot and Slide positions - only available if present not in motion **/
            double pivotPower = -gamepad2.left_stick_y * 0.7;  //should || be && ????
            if (pivotJoint.getPivotPos() < pivotJoint.POS_LIMIT_HIGH || pivotJoint.getPivotPos() > pivotJoint.POS_LIMIT_LOW) {
                pivotJoint.manualPivotRotate(pivotPower);
            } else {
//                pivotJoint.moveToCountPosition(pivotJoint.getPivotPos(), .2);
                pivotJoint.manualPivotRotate(0);
            }

            double slidePower = -0.5 * gamepad2.right_stick_y;
            if (slideSys.getSlidePos() < slideSys.EXTEND_LIMIT || slideSys.getSlidePos() > slideSys.RETRACT_LIMIT) {
                slideSys.manualSlideMove(slidePower);
            } else {
//                slideSys.moveToCountPosition(slideSys.getSlidePos(), .2);
                slideSys.manualSlideMove(0);
            }

            pivotJoint.ffRotate(kf);

            telemetry.addData("kf", "%5.2f", kf);
            telemetry.addData("Current Angle", "%5.1f", currentAngle);
            telemetry.addLine("\n--------- Slide Data ----------");
            telemetry.addData("Position Counts", slideSys.getSlidePos());

            telemetry.addLine("\n--------- Pivot Data ----------");
            telemetry.addData("Position Counts", pivotJoint.getPivotPos());
            telemetry.addData("Motor Power", "%5.2f", pivotJoint.getPivotPower());
            telemetry.update();
        }
    }

}
