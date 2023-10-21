package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import team packages
import org.firstinspires.ftc.teamcode.yise.liftArm;
import org.firstinspires.ftc.teamcode.yise.centerStageDrivFieldOrientationClass;


@TeleOp(name="FieldOrientationWithClass", group="Linear Opmode")
public class FieldOrientationClass extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public boolean canSwitchModes = false;

    @Override
    public void runOpMode() {

        // create instance of drive class
        centerStageDrivFieldOrientationClass drive = new centerStageDrivFieldOrientationClass(hardwareMap);
        // create instance of lift arm class
        liftArm arm = new liftArm(hardwareMap);

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
            //if Y toggle speed mode
            if (gamepad1.y && (drive.currentSpeed == centerStageDrivFieldOrientationClass.Speeds.NORMAL) && canSwitchModes) {
                drive.setSlowMode();
            } else if (gamepad1.y && (drive.currentSpeed == centerStageDrivFieldOrientationClass.Speeds.SLOW) && canSwitchModes) {
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

            if(gamepad1.b){
            drive.lockToRotation(gamepad1);
            }

            arm.intakeSystemIN(gamepad2);
            arm.intakeSystemOUT(gamepad2);

            // Stop the slide and keep it from holding position
            if (!arm.handStatusBusy()) {
                arm.holdPositionHand();
            }

            telemetry.addData("Hand position", arm.getHandPosition());
            telemetry.addData("intake power", arm.intakePower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
            telemetry.update();
        }
    }}


