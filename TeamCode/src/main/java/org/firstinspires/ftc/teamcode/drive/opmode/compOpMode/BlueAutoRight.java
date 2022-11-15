package org.firstinspires.ftc.teamcode.drive.opmode.compOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Bases.BaseOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Blue Auto Right", group = "drive")
public class BlueAutoRight extends BaseOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        GetHardware();

        //The coordinates are measured in inches from the center of the robot/odometry wheels
        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    //add motor movement
                })
                .turn(Math.toRadians(-41))
                .addDisplacementMarker(() -> {
                    //add motor movement
                })
                .waitSeconds(2)
                .turn(Math.toRadians(41))
                .forward(57)
                .turn(Math.toRadians(105))
                .addDisplacementMarker(() -> {
                    //add motor movement
                })
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