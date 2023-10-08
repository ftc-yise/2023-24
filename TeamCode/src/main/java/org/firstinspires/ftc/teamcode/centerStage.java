package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import team class
import org.firstinspires.ftc.teamcode.yise.centerStageDrivClass;


@TeleOp(name="Center Stage", group="Linear Opmode")
public class centerStage extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    public boolean canSwitchModes = false;


    @Override
    public void runOpMode() {

        // create instance of drive class
        centerStageDrivClass drive = new centerStageDrivClass(hardwareMap);


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


// If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }


            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}


