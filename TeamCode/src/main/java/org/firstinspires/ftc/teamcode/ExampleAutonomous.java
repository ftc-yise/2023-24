package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Example Auto", group="Linear Opmode")
@Disabled
public class ExampleAutonomous extends LinearOpMode {
    //Initialize RR
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Bot starting position
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.splineTo(new Vector2d(20, 9), Math.toRadians(45))
                //.forward(0)
                //.strafeRight(0)
                .build();



        //Follow trajectories in order
        drive.followTrajectory(traj1);
    }
}
