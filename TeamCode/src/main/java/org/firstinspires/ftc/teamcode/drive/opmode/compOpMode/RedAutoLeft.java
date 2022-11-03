package org.firstinspires.ftc.teamcode.drive.opmode.compOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Red Auto Left", group = "drive")
public class RedAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //The coordinates are measured in inches from the center of the robot/odometry wheels
        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-41))
                .waitSeconds(2)
                .turn(Math.toRadians(41))
                .forward(57)
                .turn(Math.toRadians(105))
                .waitSeconds(2)
                .turn(Math.toRadians(75))
                .forward(32)
                .build();

        /*TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1)
                .build();

        drive.turn(Math.toRadians(-41));
        drive.turn(Math.toRadians(41));
        drive.followTrajectorySequence(traj1);*/
    }
}