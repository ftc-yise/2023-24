package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;
import org.firstinspires.ftc.teamcode.yise.RobotNavigation;

@TeleOp(name="1 Controller Drive", group="Linear Opmode")
public class SubsystemTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;

    boolean canToggleHandPosition = true;
    boolean handPositionIn = true;

    @Override
    public void runOpMode() {
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);
        //Instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * Hand
             */
            if (!gamepad1.x) {
                canToggleHandPosition = true;
            }

            if (gamepad1.x && canToggleHandPosition) {
                canToggleHandPosition = false;
                if (arm.handPosition == LiftArm.HandPosition.IN){
                    arm.setHandPosition(LiftArm.HandPosition.OUT);
                    handPositionIn = true;
                } else {
                    arm.setHandPosition(LiftArm.HandPosition.IN);
                    handPositionIn = false;
                }
            }



            /**
             * Driving
             */
            // If we have any Dpad input, update the motor power based on dpad

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double theta = orientation.getYaw(AngleUnit.RADIANS) + Math.PI;

            rrDrive.updateMotorsFromStick(gamepad1);
            rrDrive.update();


            //For testing
            if (gamepad1.dpad_left) {
                rrDrive.drive10in();
            }


            /**
             * Intake
             */
            if (gamepad1.right_trigger > 0.75){
                intakeSystem.runIntakeSystem(1);
            } else if (gamepad1.left_trigger > 0.75){
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                intakeSystem.runIntakeSystem(0);
            }



            /**
             * Arm slides
             */
            if (gamepad1.dpad_up){
                arm.setArmDistance(LiftArm.Distance.FULL);
            } else if (gamepad1.dpad_right){
                arm.setArmDistance(LiftArm.Distance.HALF);
            } else if (gamepad1.dpad_down) {
                arm.setArmDistance(LiftArm.Distance.DEFAULT);
            }



            /**
             * Trapdoor
             */
            if (gamepad1.left_bumper) {
                arm.closeTrapdoor();
            } else if (gamepad1.right_bumper) {
                arm.openTrapdoor();
            }


            /**
             * Slow mode toggle
             */
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;

                //Toggle between slow and normal speeds
                switch (rrDrive.currentSpeed) {
                    case SLOW:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        rrDrive.toggleSlowMode(RoadRunnerDriving.Speeds.SLOW);
                        break;
                }
            }

            /**
             * Telemetry data
             */
            telemetry.addData("Trapdoor: ", arm.trapdoor.getPosition());
            telemetry.addData("Hand: ", arm.getHandPosition());
            telemetry.addData("Intake: ", arm.intakePower);
            telemetry.addData("Slides (R, L): ", arm.getSlidePosition() + ", " + arm.getSlidePosition());

            telemetry.addLine();

            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}


