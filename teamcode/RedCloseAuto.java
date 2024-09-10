package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class RedCloseAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        SampleMecanumDrive autoDrive = new SampleMecanumDrive(hardwareMap);

        robot.resetArmSystemEncoders();
//        robot.armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d startLocation = new Pose2d(15.0, -65.0, Math.toRadians(90));
        TrajectorySequence centerSpike = autoDrive.trajectorySequenceBuilder(startLocation)
                .setReversed(false)
                .setReversed(false)
                .splineTo(new Vector2d(15, -38), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, ()->robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->robot.wristServo.setPosition(robot.SERVOE_SCORE_AUTO))
                .waitSeconds(1)
                .back(8)
                .lineToSplineHeading(new Pose2d(54, -36, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.pivotMoveToCountPosition(robot.PIVOT_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.slideMoveToCountPosition(robot.SLIDE_POS_AUTO, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(3.6)
                .strafeRight(32)
                .forward(14)
                .build();
        waitForStart();
                autoDrive.setPoseEstimate(startLocation);
                autoDrive.followTrajectorySequence(centerSpike);

        }

}
