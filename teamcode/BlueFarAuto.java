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

public class BlueFarAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        SampleMecanumDrive autoDrive = new SampleMecanumDrive(hardwareMap);

        robot.resetArmSystemEncoders();
//        robot.armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d startLocation = new Pose2d(-40, 65.0, Math.toRadians(-90));
        TrajectorySequence centerSpike = autoDrive.trajectorySequenceBuilder(startLocation)


                .setReversed(false)
                .splineTo(new Vector2d(-40, 38), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, ()->robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.rightToggle())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(1)
                //pick up white pixel
                .splineTo(new Vector2d(-15, 10), Math.toRadians(0))
                .splineTo(new Vector2d(50, 36), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.pivotMoveToCountPosition(400, 1))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.wristServo.setPosition(robot.SERVO_FLOOR_DROP))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.leftToggle())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.slideMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3, ()->robot.pivotMoveToCountPosition(0, 1))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->robot.wristServo.setPosition(robot.SERVO_NEST))
                .waitSeconds(4)
                .strafeLeft(30)
                .forward(14)
                .build();


        waitForStart();
                autoDrive.setPoseEstimate(startLocation);
                autoDrive.followTrajectorySequence(centerSpike);

        }
}

//OLD CODE

//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.wristServo.setPosition(0.5))
//                .waitSeconds(1)
//                .forward(26)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->robot.rightToggle())//1
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.rightToggle())//1.5
//                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->robot.wristServo.setPosition(0.95))//2
//                .waitSeconds(1.5)
//                .forward(30)
//                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->robot.encoderPivot(755))
//                .waitSeconds(1.5)
//                .turn(Math.toRadians(110))
//                .forward(86)
//                .strafeLeft(30)
//                .forward(3)
//                //.splineTo(new Vector2d(40,36 ),Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> robot.encoderPivot(1400))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.encoderSlide(800))
//                .UNSTABLE_addTemporalMarkerOffset(2, ()->robot.wristServo.setPosition(0.4))
//                .UNSTABLE_addTemporalMarkerOffset(2.5, ()-> robot.leftToggle())
//                .UNSTABLE_addTemporalMarkerOffset(3.5, ()-> robot.leftToggle())
//                .UNSTABLE_addTemporalMarkerOffset(4, ()->robot.wristServo.setPosition(0.95))
//                .waitSeconds(5)
//                .back(10)
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.encoderSlide(0))
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()-> robot.encoderPivot(0))
//               .waitSeconds(2)
//                .strafeRight(40)
//                .forward(15)
//.splineTo(new Vector2d(50,60 ),Math.toRadians(180))

