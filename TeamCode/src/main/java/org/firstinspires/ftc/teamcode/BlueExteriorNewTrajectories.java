package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.PoseStorage;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;


@Autonomous(name="Blue Exterior New Traj Test", group="Linear Opmode")
public class BlueExteriorNewTrajectories extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int prop;

    @Override
    public void runOpMode() {
        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        IntakeSystem intake = new IntakeSystem(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        //Sense cones
        while (!isStarted()) {
            prop = vision.getPropPosition();

            telemetry.addData("Prop: ", prop);
            telemetry.update();
        }

        if (isStopRequested()) return;


        //Bot starting position
        Pose2d startPose = new Pose2d(-39.75, 61.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions


        TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30, 43, Math.toRadians(120)))
                .build();

        TrajectorySequence dropPixelCenter = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-41, 35, Math.toRadians(90)))
                .build();

        TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, 43, Math.toRadians(60)))
                .build();

        TrajectorySequence driveToPixelStackLeft = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                .strafeLeft(10)
                .splineTo(new Vector2d(-50, 12), Math.toRadians(-90))
                .forward(7)
                .build();

        TrajectorySequence driveToPixelStackCenter = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                .lineToLinearHeading(new Pose2d(-54, 43, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-54, 12, Math.toRadians(180)))
                .forward(3)
                .build();

        TrajectorySequence driveToPixelStackRight = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                .lineToLinearHeading(new Pose2d(-30, 12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(180)))
                .build();

        TrajectorySequence driveToBoardLeft = drive.trajectorySequenceBuilder(driveToPixelStackLeft.end())
                .back(50)
                .splineTo(new Vector2d(44, 44), Math.toRadians(0))
                .addDisplacementMarker(50, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence driveToBoardCenter = drive.trajectorySequenceBuilder(driveToPixelStackCenter.end())
                .back(50)
                .splineTo(new Vector2d(44, 38), Math.toRadians(0))
                .addDisplacementMarker(50, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence driveToBoardRight = drive.trajectorySequenceBuilder(driveToPixelStackRight.end())
                .back(50)
                .splineTo(new Vector2d(44, 32), Math.toRadians(0))
                .addDisplacementMarker(50, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(2)
                .build();


        //Follow trajectories in order

        /* Left */
        if(prop == 2) {
            drive.followTrajectorySequence(dropPixelLeft);
            sleep(500);
            drive.followTrajectorySequence(driveToPixelStackLeft);

            intake.runIntakeSystem(1);
            sleep(3000);
            intake.runIntakeSystem(0);

            drive.followTrajectorySequence(driveToBoardLeft);
            arm.openTrapdoor();
            sleep(1000);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            //drive.followTrajectorySequence(park);
            PoseStorage.currentPose = drive.getPoseEstimate();

        } /* Middle */
        else if (prop == 1) {

            drive.followTrajectorySequence(dropPixelCenter);
            sleep(500);
            drive.followTrajectorySequence(driveToPixelStackCenter);

            intake.runIntakeSystem(1);
            sleep(3000);
            intake.runIntakeSystem(0);

            drive.followTrajectorySequence(driveToBoardCenter);
            arm.openTrapdoor();
            sleep(1000);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            //drive.followTrajectorySequence(park);
            PoseStorage.currentPose = drive.getPoseEstimate();

        } /* Right */
        else if (prop == 0) {
            drive.followTrajectorySequence(dropPixelRight);
            sleep(500);
            drive.followTrajectorySequence(driveToPixelStackRight);

            intake.runIntakeSystem(1);
            sleep(3000);
            intake.runIntakeSystem(0);

            drive.followTrajectorySequence(driveToBoardRight);
            arm.openTrapdoor();
            sleep(1000);
            arm.closeTrapdoor();
            arm.retract();
            sleep(2000);
            //drive.followTrajectorySequence(park);
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}
