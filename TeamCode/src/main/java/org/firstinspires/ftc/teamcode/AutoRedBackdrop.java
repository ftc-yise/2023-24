package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red Backdrop", group="Linear Opmode")
public class AutoRedBackdrop extends LinearOpMode {

    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Bot starting position
        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.lineToConstantHeading(new Vector2d(12, -33))
                .strafeRight(29)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //.lineToConstantHeading(new Vector2d(12, -53))
                .forward(33)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //.splineToLinearHeading(new Pose2d(48, -36, Math.toRadians(0)), 0)
                .strafeLeft(25)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //.strafeRight(25)
                .forward(10)
                .build();


        //Follow trajectories in order
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
    }
}
