package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.RobotNavigation;

@TeleOp(name="1 Controller Drive", group="Linear Opmode")
public class SubsystemTest extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;

    boolean canToggleHandPosition = true;
    boolean handPositionIn = true;

    boolean fieldOrientation = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        //RobotNavigation drive = new RobotNavigation(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);

        SampleMecanumDrive rrDrive = new SampleMecanumDrive(hardwareMap);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        arm.openIntakeHolder();

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

            /*if (!arm.handStatusBusy() && handPositionIn) {
                arm.holdPositionHandIN();
            } else if (!arm.handStatusBusy() && !handPositionIn){
                arm.holdPositionHandOUT();
            }*/

            /**
             * Driving
             */
            //Enable field orientation through button
            if (gamepad1.back) {
                fieldOrientation = true;
            }

            // If we have any Dpad input, update the motor power based on dpad
            /*if (fieldOrientation) {
                drive.updateMotorsFieldOrientation(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }*/

            rrDrive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

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

            if (gamepad2.a){
                arm.openIntakeHolder();
            }


            /**
             * Slow mode toggle
             */
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            /*if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;

                //Toggle between slow and normal speeds
                switch (drive.currentSpeed) {
                    case SLOW:
                        drive.toggleSlowMode(RobotNavigation.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        drive.toggleSlowMode(RobotNavigation.Speeds.SLOW);
                        break;
                }
            }*/

            if (!arm.slideStatusBusy()) {
                arm.holdPosition();
            }


            /**
             * Telemetry data
             */
            telemetry.addData("Trapdoor: ", arm.trapdoor.getPosition());
            telemetry.addData("Hand: ", arm.getHandPosition());
            telemetry.addData("Intake: ", arm.intakePower);
            telemetry.addData("Slides (R, L): ", arm.getSlidePosition(LiftArm.Sides.RIGHT) + ", " + arm.getSlidePosition(LiftArm.Sides.LEFT));

            telemetry.addLine();

            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}


