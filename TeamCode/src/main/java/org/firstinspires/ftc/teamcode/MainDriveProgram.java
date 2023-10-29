package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import team class
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.RobotNavigation;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

@TeleOp(name="Center Stage", group="Linear Opmode")
public class MainDriveProgram extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    boolean canToggleSlowMode = true;
    boolean canToggleHandPosition = true;
    boolean fieldOrientation = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        RobotNavigation drive = new RobotNavigation(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);
        // create instance of tensorflow vision class
        TensorflowVision tfodVision = new TensorflowVision(hardwareMap);

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

                //Toggle between slow and normal speeds
                switch (arm.handPosition) {
                    case IN:
                        arm.setHandPosition(LiftArm.HandPosition.OUT);
                        break;
                    case OUT:
                        arm.setHandPosition(LiftArm.HandPosition.IN);
                        break;
                }
            }



            /**
             * Driving
             */
            //Enable field orientation through button
            if (gamepad1.back) {
                fieldOrientation = true;
            }

            // If we have any Dpad input, update the motor power based on dpad
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else if (fieldOrientation) {
                drive.updateMotorsFieldOrientation(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }



            /**
             * Intake
             */
            if (gamepad2.a){
                intakeSystem.runIntakeSystem(1);
            } else if (gamepad2.b){
                intakeSystem.runIntakeSystem(-0.5);
            } else {
                intakeSystem.runIntakeSystem(0);
            }



            /**
             * Arm slides
             */
            if (gamepad2.dpad_up){
                arm.setArmDistance(LiftArm.Distance.HALF);
            } else if (gamepad2.dpad_down){
                arm.setArmDistance(LiftArm.Distance.DEFAULT);
            }



            /**
             * Trapdoor
             */
            if (gamepad2.left_bumper) {
                arm.closeTrapdoor();
            } else if (gamepad2.right_bumper) {
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
                switch (drive.currentSpeed) {
                    case SLOW:
                        drive.toggleSlowMode(RobotNavigation.Speeds.NORMAL);
                        break;
                    case NORMAL:
                        drive.toggleSlowMode(RobotNavigation.Speeds.SLOW);
                        break;
                }
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

            telemetry.addLine();

            telemetry.addData("Prop position: ", tfodVision.getPropPosition());
            telemetry.update();
        }
    }
}


