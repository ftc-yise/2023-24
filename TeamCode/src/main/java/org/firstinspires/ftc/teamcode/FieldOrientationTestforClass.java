package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.yise.centerStageDrivFieldOrientationClass;


@TeleOp(name="FieldOrientationWithClass", group="Linear Opmode")
public class FieldOrientationTestforClass extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public boolean canSwitchModes = false;


    @Override
    public void runOpMode() {

        // create instance of drive class
        centerStageDrivFieldOrientationClass drive = new centerStageDrivFieldOrientationClass(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        /*RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);*/
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!gamepad1.y) {
                canSwitchModes = true;
            }
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


// If we have any Dpad input, update the motor power based on Dpad (ie overright stick)
            if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down) {
                drive.updateMotorsFromDpad(gamepad1);
            } else {
                drive.updateMotorsFromStick(gamepad1);
            }

            if(gamepad1.b){
            drive.lockToRotation(gamepad1);
            }


            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
            telemetry.update();
        }
    }}


