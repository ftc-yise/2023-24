package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftArm {
    public DcMotor leftSlide, rightSlide, hand;
    public Servo trapdoor;

    public HandPosition handPosition;
    public double intakePower = 0;

    // Used to set pole height.
    public enum Distance {
        DEFAULT,
        HALF,
        FULL
    }

    public enum HandPosition{
        IN,
        OUT
    }


    public enum TrapdoorPositions{
        OPEN,
        CLOSE
    }

    // Used to identify left and right slide motors
    public enum Sides {
        RIGHT,
        LEFT
    }


    //Constructor
    public LiftArm(HardwareMap hardwareMap) {
        //Initialize motors and servos
        hand = hardwareMap.get(DcMotor.class, "hand");
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");


        trapdoor.setPosition(Servo.MIN_POSITION);

        //Set motor directions
        hand.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        //Reset encoders
        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setHandPosition(HandPosition.IN);
    }



    public void setArmDistance(Distance targetDistance) {
        switch (targetDistance) {
            case DEFAULT:
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);
                break;
            case HALF:
                leftSlide.setTargetPosition(1100);
                rightSlide.setTargetPosition(1100);
                break;
            case FULL:
                leftSlide.setTargetPosition(1950);
                rightSlide.setTargetPosition(1950);
                break;
        }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public boolean slideStatusBusy() {
        boolean busy = false;
        if (rightSlide.isBusy() || leftSlide.isBusy()) {
            busy = true;
        }
        return busy;
    }

    public boolean handStatusBusy() {
        boolean busyHand = false;
        if (hand.isBusy()) {
            busyHand = true;
        }
        return busyHand;
    }

    public void setHandPosition(HandPosition targetHandPosition) {
        handPosition = targetHandPosition;
        switch (targetHandPosition) {
            case IN:
                hand.setTargetPosition(-5);
                break;
            case OUT:
                hand.setTargetPosition(-150);
                break;
        }
        hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hand.setPower(0.5);
    }



    public void closeTrapdoor() {
        trapdoor.setPosition(1);
    }
    public void openTrapdoor() {
        trapdoor.setPosition(Servo.MIN_POSITION);
    }

    public void holdPositionHandOUT() {
        hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hand.setPower(0.05);
    }

    public void holdPositionHandIN() {
        hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hand.setPower(-.25);
    }


    public double getSlidePosition(Sides side) {
        if (side == Sides.LEFT) {
            return leftSlide.getCurrentPosition();
        } else if (side == Sides.RIGHT) {
            return rightSlide.getCurrentPosition();
        }

        return 0;
    }

    public void holdPosition() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0.05);
        rightSlide.setPower(0.05);
    }

    public double getHandPosition() {
        return hand.getCurrentPosition();
    }
}