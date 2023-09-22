package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp(name="FieldOrientation", group="Linear Opmode")
public class FieldOrientation extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public float speedmulti = 0.2f;
    public float Yout = 0;

    IMU imu;


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
        imu = hardwareMap.get(IMU.class, "imu");


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

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);





            if(gamepad1.y && speedmulti == 1){
                speedmulti = .25f;
            } else if(gamepad1.y && speedmulti == .5f){
                speedmulti = 1f;
            }

            telemetry.addData("motor PowerLB:", leftBackDrive.getPower());
            telemetry.addData("motor PowerLF:", leftFrontDrive.getPower());
            telemetry.addData("motor PowerRF:", rightFrontDrive.getPower());
            telemetry.addData("motor PowerRB:", rightBackDrive.getPower());


            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double horizontal =  gamepad1.left_stick_x * -1;
            double vertical     =  gamepad1.left_stick_y * -1;
            double turn     =  gamepad1.right_stick_x;






            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            /*double leftFrontPower  = vertical - horizontal - turn * speedmulti;
            double rightFrontPower = vertical + horizontal + turn * speedmulti;
            double leftBackPower   = vertical + horizontal - turn * speedmulti;
            double rightBackPower  = vertical - horizontal + turn * speedmulti;*/

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            /*max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));*/

            

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);



            double theta = orientation.getYaw(AngleUnit.DEGREES);

            if (gamepad1.a) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press A (triangle) on Gamepad to reset\n");
            }

            theta = theta +180;




            double HorizontalOut = horizontal * Math.cos(Math.toRadians(theta)) - vertical * (Math.sin(Math.toRadians(theta)));
            double VerticalOut = vertical * Math.cos(Math.toRadians(theta)) + horizontal * Math.sin(Math.toRadians(theta));


            double leftFrontPower  = VerticalOut - HorizontalOut - turn * speedmulti;
            double rightFrontPower = VerticalOut + HorizontalOut + turn * speedmulti;
            double leftBackPower   = VerticalOut + HorizontalOut - turn * speedmulti;
            double rightBackPower  = VerticalOut - HorizontalOut + turn * speedmulti;


            /*max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }*/







            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.



            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);




            telemetry.addData("theta", "%.2f Deg. (Heading)", theta);

            telemetry.addData("horizontal", gamepad1.left_stick_x * -1);
            telemetry.addData("vertical", gamepad1.left_stick_y * -1);
            telemetry.addData("turn", gamepad1.right_stick_x);


            telemetry.addData("verticle out", VerticalOut);
            telemetry.addData("horizontal out", HorizontalOut);


            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES) + 180);
          /*  telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);*/
            telemetry.update();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            /*telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);*/
            //telemetry.addData("limitSwitchState", limitSwitchState);


            //telemetry.addData("CR positon",fireServo.getPosition());
            //telemetry.addData("CR class",fireServo.getClass());
            telemetry.update();
        }
    }}


