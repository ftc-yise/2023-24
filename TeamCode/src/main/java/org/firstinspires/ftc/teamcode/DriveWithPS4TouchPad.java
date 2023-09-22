package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TouchPad Drive", group="Linear Opmode")
public class DriveWithPS4TouchPad extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public float speedmulti = 1;


    //private Servo fireServo = null;
    //private CRServo barrel = null;
    TouchSensor limit;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        //fireServo = hardwareMap.get(Servo.class, "fire_servo");
        //barrel = hardwareMap.get(CRServo.class, "barrel" );

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //fireServo.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        /*if (limit.isPressed()) {
            leftFrontDrive.setPower(0);
        } else { // Otherwise, run the motor
            leftFrontDrive.setPower(1);
        }*/


        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Updated upstream
            //Manual slide

            TouchSensor limit = null;

            /*if (limit.isPressed()) {
                leftBackDrive.setPower(0);
            } else { // Otherwise, run the motor
                leftBackDrive.setPower(1);
            }*/

            if(gamepad1.y && speedmulti == 1){
                speedmulti = .5f;
            } else if(gamepad1.y && speedmulti == .5f){
                speedmulti = 1f;
            }

            telemetry.addData("motor Power:", leftBackDrive.getPower());


            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double horizontal =  gamepad1.touchpad_finger_1_x;
            double vertical     =  gamepad1.touchpad_finger_1_y;
            double turn     =  gamepad1.touchpad_finger_2_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = vertical - horizontal - turn * speedmulti;
            double rightFrontPower = vertical + horizontal + turn * speedmulti;
            double leftBackPower   = vertical + horizontal - turn * speedmulti;
            double rightBackPower  = vertical - horizontal + turn * speedmulti;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            
            /* verticle =y
               horizontal = x
               turn = r*/
            Math.sin(vertical);

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            /*if(gamepad1.right_stick_x > .5){
                leftFrontPower = -1 * speedmulti;
                leftBackPower = -1 * speedmulti;
                rightBackPower = 1 * speedmulti;
                rightFrontPower = 1 * speedmulti;
            }else if(gamepad1.right_stick_x  < -.5){
                leftFrontPower = 1 * speedmulti;
                leftBackPower = 1 * speedmulti;
                rightBackPower = -1 * speedmulti;
                rightFrontPower = -1 * speedmulti;
            }
            if(gamepad1.left_stick_x < -.5){
                leftFrontPower = 1 * speedmulti;
                leftBackPower = -1 * speedmulti;
                rightBackPower = -1 * speedmulti;
                rightFrontPower = 1 * speedmulti;
            }else if(gamepad1.left_stick_x  > .5){
                leftFrontPower = -1 * speedmulti;
                leftBackPower = 1 * speedmulti;
                rightBackPower = 1 * speedmulti;
                rightFrontPower = -1 * speedmulti;
            }*/
                      /*else if(gamepad1.right_trigger > .5){
                sholder.setD;
                elbow.setPosition();
            }else if(gamepad1.left_bumper){
                sholder.setPosition(Servo.MIN_POSITION);
                elbow.setPosition(Servo.MIN_POSITION);
            }*/
            // testing out how to make the motors stop on a dime by keeping constant power so they dont spin freely
           /* if(gamepad1.left_stick_x == 0){
                leftFrontPower = .01;
                leftBackPower = -.01;
                rightBackPower = -.01;
                rightFrontPower = .01;
            }else if(gamepad1.right_stick_x == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }else if(gamepad1.right_stick_y == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }else if(gamepad1.left_stick_y == 0) {
                leftFrontPower = -.01;
                leftBackPower = .01;
                rightBackPower = .01;
                rightFrontPower = -.01;
            }*/
            /*if (gamepad1.a){
                fireServo.setPosition(Servo.MAX_POSITION);
            }
            else if (gamepad1.b){
                fireServo.setPosition(Servo.MIN_POSITION);

            }

            if (gamepad1.left_bumper){
                barrel.setPower(1);

            }else if (gamepad1.right_bumper){
                barrel.setPower(-1);

            }
            else
                barrel.setPower(0);








            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("limitSwitchState", limitSwitchState);


            //telemetry.addData("CR positon",fireServo.getPosition());
            //telemetry.addData("CR class",fireServo.getClass());
            telemetry.update();
        }
    }}


