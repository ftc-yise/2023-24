package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class LiftArm {
    public DcMotor slide, hand;
    public Servo trapdoor;
    public Servo intakeHolder;

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

    public enum holderPositions{
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
        slide = hardwareMap.get(DcMotor.class, "slide");
        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        intakeHolder = hardwareMap.get(Servo.class, "intakeHolder");

        intakeHolder.setDirection(Servo.Direction.REVERSE);

        trapdoor.setPosition(.2);

        //Set motor directions
        hand.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);

        //Reset encoders
        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setHandPosition(HandPosition.IN);
    }



    public void setArmDistance(Distance targetDistance) {
        switch (targetDistance) {
            case DEFAULT:
                slide.setTargetPosition(0);
                break;
            case HALF:
                slide.setTargetPosition(1200);
                break;
            case FULL:
                slide.setTargetPosition(2400);
                break;
        }
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        if (!slide.isBusy()) {
            slide.setPower(0.05);
        }
    }



    public void setHandPosition(HandPosition targetHandPosition) {
        handPosition = targetHandPosition;
        switch (targetHandPosition) {
            case IN:
                hand.setTargetPosition(0);
                break;
            case OUT:
                hand.setTargetPosition(-120);
                break;
        }
        hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //120-0
        hand.setPower(Math.abs((hand.getTargetPosition() - hand.getCurrentPosition())/120));
        if (!hand.isBusy()) {
            hand.setPower(0.05);
        }
    }



    public void openTrapdoor() {
        trapdoor.setPosition(0.6);
    }
    public void closeTrapdoor() {
        trapdoor.setPosition(0.2);
    }


    public double getSlidePosition() {
        return slide.getCurrentPosition();
    }

    public double getHandPosition() {
        return hand.getCurrentPosition();
    }
}