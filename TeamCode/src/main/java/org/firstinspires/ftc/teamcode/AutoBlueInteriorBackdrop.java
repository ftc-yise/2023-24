package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

//import team class
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Blue Park Interior", group="Linear Opmode")
public class AutoBlueInteriorBackdrop extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int Prop;
    float propLocation = 2f;

    @Override
    public void runOpMode() {
        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        //Sense cones
        while (!isStarted()) {
            sleep(2000);
            Prop = vision.getPropPosition();

            telemetry.addData("Prop: ", Prop);
            telemetry.update();
        }

        if (isStopRequested()) return;


        //Bot starting position
        Pose2d startPose = new Pose2d(9, 61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions

        Trajectory traj1_1 = drive.trajectoryBuilder(new Pose2d(9, -61, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(5, 35, Math.toRadians(90)))
                .back(-5)
                .build();

        Trajectory traj1_2 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15, 30, Math.toRadians(90)))
                .back(-5)
                .build();

        Trajectory traj1_3 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(25, 42, Math.toRadians(90)))
                .build();

        Trajectory traj2_1_1 = drive.trajectoryBuilder(traj1_1.end())
                .lineToLinearHeading(new Pose2d(38, 68, Math.toRadians(180)))
                .build();
        Trajectory traj2_1_2 = drive.trajectoryBuilder(new Pose2d(38, -64, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(39, 34, Math.toRadians(180)))
                .build();

        Trajectory traj2_3_1 = drive.trajectoryBuilder(traj1_3.end())
                .back(5)
                .build();


        Trajectory traj2_2_1 = drive.trajectoryBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(38, 68, Math.toRadians(180)))
                .build();

        Trajectory traj2_2_2 = drive.trajectoryBuilder(new Pose2d(38, 64, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(45, 34, Math.toRadians(180)))
                .addDisplacementMarker(2, () -> {
                    arm.setArmDistance(LiftArm.Distance.HALF);
                    arm.setHandPosition(LiftArm.HandPosition.OUT);
                })
                .build();



        Trajectory traj3_1 = drive.trajectoryBuilder(traj2_1_2.end())
                .forward(5)
                .addDisplacementMarker(2, arm::openTrapdoor)
                .build();
        Trajectory traj4_1 = drive.trajectoryBuilder(traj3_1.end())
                .forward(-10)
                .strafeLeft(-28)
                .forward(18)
                .addDisplacementMarker(2, () -> {
                    arm.setHandPosition(LiftArm.HandPosition.IN);
                    arm.setArmDistance(LiftArm.Distance.DEFAULT);
                    arm.closeTrapdoor();
                })
                .build();

        Trajectory traj3_2 = drive.trajectoryBuilder(traj2_2_2.end())
                .forward(5)
                .addDisplacementMarker(2, arm::openTrapdoor)
                .build();
        Trajectory traj4_2 = drive.trajectoryBuilder(traj3_2.end())
                .forward(-10)
                .strafeLeft(-28)
                .forward(18)
                .addDisplacementMarker(2, () -> {
                    arm.setHandPosition(LiftArm.HandPosition.IN);
                    arm.setArmDistance(LiftArm.Distance.DEFAULT);
                    arm.closeTrapdoor();
                })
                .build();

        Trajectory traj2_3_2 = drive.trajectoryBuilder(traj2_3_1.end())
                .lineToLinearHeading(new Pose2d(44, 68, Math.toRadians(180)))
                .addDisplacementMarker(2, () -> {
                    arm.setArmDistance(LiftArm.Distance.HALF);
                    arm.setHandPosition(LiftArm.HandPosition.OUT);
                })
                .build();

        Trajectory traj3_3 = drive.trajectoryBuilder(new Pose2d(38, 64, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(51, 34, Math.toRadians(180)))
                .addDisplacementMarker(2, () -> {
                    arm.setArmDistance(LiftArm.Distance.HALF);
                    arm.setHandPosition(LiftArm.HandPosition.OUT);
                })
                .build();
        Trajectory traj4_3 = drive.trajectoryBuilder(traj3_3.end())
                .forward(10)
                .addDisplacementMarker(2, arm::openTrapdoor)
                .build();
        Trajectory traj5_3 = drive.trajectoryBuilder(traj4_3.end())
                .forward(-10)
                .strafeLeft(-28)
                .forward(18)
                .addDisplacementMarker(0.2, () -> {
                    arm.setHandPosition(LiftArm.HandPosition.IN);

                })
                .addDisplacementMarker(2, () -> {
                    arm.setArmDistance(LiftArm.Distance.DEFAULT);
                    arm.closeTrapdoor();
                })
                .build();

        //Follow trajectories in order
        //switch between parking
        if(vision.getPropPosition() == 2) {
            sleep(1000);
            drive.followTrajectory(traj1_1);
            sleep(1000);
            drive.followTrajectory(traj2_1_1);
            sleep(1000);
            drive.followTrajectory(traj2_1_2);
            sleep(1000);
            drive.followTrajectory(traj3_1);
            sleep(1000);
            drive.followTrajectory(traj4_1);
        } else if (vision.getPropPosition() == 1) {
            sleep(1000);
            drive.followTrajectory(traj1_2);
            sleep(2000);
            drive.followTrajectory(traj2_2_1);
            sleep(2000);
            drive.followTrajectory(traj2_2_2);
            sleep(2000);
            drive.followTrajectory(traj3_2);
            sleep(2000);
            drive.followTrajectory(traj4_2);
        } else if (vision.getPropPosition() == 0) {
            sleep(1000);
            drive.followTrajectory(traj1_3);
            sleep(3000);
            drive.followTrajectory(traj2_3_1);
            sleep(3000);
            drive.followTrajectory(traj2_3_2);
            sleep(3000);
            drive.followTrajectory(traj3_3);
            sleep(6000);
            drive.followTrajectory(traj4_3);
            drive.followTrajectory(traj5_3);
        }
    }
}
