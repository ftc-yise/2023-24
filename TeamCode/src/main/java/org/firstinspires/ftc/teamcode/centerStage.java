package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import team class
import org.firstinspires.ftc.teamcode.yise.centerStageDrivClass;
import org.firstinspires.ftc.teamcode.yise.centerStageDrivFieldOrientationClass;
import org.firstinspires.ftc.teamcode.yise.liftArm;

@TeleOp(name="Center Stage", group="Linear Opmode")
public class centerStage extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    public boolean canSwitchModes = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        centerStageDrivClass drive = new centerStageDrivClass(hardwareMap);
        // create instance of lift arm class
        liftArm arm = new liftArm(hardwareMap);

        arm.setHandPosition(liftArm.Hand.IN);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!gamepad1.y) {
                canSwitchModes = true;
            }
            if (gamepad1.y && (drive.currentSpeed == centerStageDrivClass.Speeds.NORMAL) && canSwitchModes) {
                drive.setSlowMode();
            } else if (gamepad1.y && (drive.currentSpeed == centerStageDrivClass.Speeds.SLOW) && canSwitchModes) {
                drive.setNormalMode();
            }

            if (gamepad1.y) {
                canSwitchModes = false;
            } else {
                canSwitchModes = true;
            }

            //if X or B bring the hand IN and OUT
            if (gamepad2.x) {
                arm.setHandPosition(liftArm.Hand.OUT);
            } else if (gamepad2.b) {
                arm.setHandPosition(liftArm.Hand.IN);
            }

// If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            //the importation of the intake from class "lift arm"
            arm.intakeSystemIN(gamepad2);
            arm.intakeSystemOUT(gamepad2);

            if(gamepad2.dpad_up){
                arm.setPoleHeight(liftArm.Heights.LOW);
            } else if (gamepad2.dpad_down){
                arm.returnToBottom();
            }

            if (gamepad2.left_bumper) {
                arm.closeTrapdoor();
            } else if (gamepad2.right_bumper) {
                arm.openTrapdoor();
            }

            // Stop the slide and keep it from holding position
            if (!arm.handStatusBusy()) {
                arm.holdPositionHand();
            }

            //Data about the position of the Hand and the power going to the intake
            telemetry.addData("Hand position", arm.getHandPosition());
            telemetry.addData("intake power", arm.intakePower);
            telemetry.addData("Slides Position", arm.getSlidePosition(liftArm.Sides.RIGHT));

            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}


