package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="Center Stage", group="Linear Opmode")
public class centerDrive extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private float speedMulti = 1f;
    private boolean canChangeSpeeds = true;

    @Override
    public void runOpMode() {


        //Initialize hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");


        //Set motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.y && canChangeSpeeds){
                canChangeSpeeds = false;
                if (speedMulti == 0.4) {
                    speedMulti = 1;
                } else if (speedMulti == 1) {
                    speedMulti = 0.4f;
                }
            } else if (!gamepad1.y) {
                canChangeSpeeds = true;
            }

            telemetry.addData("motor PowerLB:", leftBackDrive.getPower());
            telemetry.addData("motor PowerLF:", leftFrontDrive.getPower());
            telemetry.addData("motor PowerRF:", rightFrontDrive.getPower());
            telemetry.addData("motor PowerRB:", rightBackDrive.getPower());



            //Angle returns -pi to pi, we need 0 to 2pi, so add pi to result



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value

            double horizontal =  gamepad1.left_stick_x;
            double vertical     =  gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;






            double leftFrontPower  = (vertical - horizontal - turn) * speedMulti;
            double rightFrontPower = (vertical + horizontal + turn) * speedMulti;
            double leftBackPower   = (vertical + horizontal - turn) * speedMulti;
            double rightBackPower  = (vertical - horizontal + turn) * speedMulti;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



            telemetry.addData("Horizontal input", gamepad1.left_stick_x);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }}


