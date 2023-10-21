package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class liftArm {
    public final DcMotor leftSlide, rightSlide, hand;
    public final DcMotor intake;
    public final Servo trapdoor;
    public double intakePower;
    // Used to set pole height.
    public enum Heights {
        LOW,
        MEDIUM,
        HIGH
    }

    public enum Hand{
        IN,
        OUT
    }

    public trapdoorPositions trapdoorStatus;
    public enum trapdoorPositions{
        OPEN,
        CLOSE
    }

    // Used to identify left and right slide motors
    public enum Sides {
        RIGHT,
        LEFT
    }

    public liftArm(HardwareMap hardwareMap) {
        hand = hardwareMap.get(DcMotor.class, "hand");
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        intake = hardwareMap.get(DcMotor.class, "intake");

        trapdoor = hardwareMap.get(Servo.class, "trapdoor");
        trapdoor.setPosition(Servo.MIN_POSITION);
        trapdoorStatus = trapdoorPositions.OPEN;

        hand.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);


        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set default power for the intake
        intakePower = 0;
    }

    public void setPoleHeight(Heights targetHeight) {
        switch (targetHeight) {
            case LOW:
                leftSlide.setTargetPosition(1100);
                rightSlide.setTargetPosition(1100);
                break;
            case MEDIUM:
                leftSlide.setTargetPosition(1400);
                rightSlide.setTargetPosition(1400);
                break;
            case HIGH:
                leftSlide.setTargetPosition(1950);
                rightSlide.setTargetPosition(1950);
                break;
        }
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(.5);
        rightSlide.setPower(.5);
    }

    public void setHandPosition(Hand targetPosition) {
        switch (targetPosition) {
            case IN:
                hand.setTargetPosition(-5);
                break;
            case OUT:
                hand.setTargetPosition(-105);
                break;
        }
        hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hand.setPower(.5);
    }


        public void returnToBottom() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(.5);
        rightSlide.setPower(.5);
    }

    public boolean forceDown() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(-0.5);
        rightSlide.setPower(-0.5);
        return false;
    }

    public boolean stopAndReset() {
        hand.setPower(0);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
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

    public void holdPositionSlides() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setPower(0.05);
        rightSlide.setPower(0.05);

    }

    public void holdPositionHand() {
        hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hand.setPower(0.05);
    }

    public void intakeSystemIN(Gamepad gamepad) {
        if (gamepad.a){
            intakePower = 1;
        }else if (!gamepad.a){
            intakePower = 0;
        }
        intake.setPower(intakePower);
    }

    public void intakeSystemOUT(Gamepad gamepad) {
        if (gamepad.y){
            intakePower = -1;
        }else if (!gamepad.y){
            intakePower = 0;
        }
        intake.setPower(intakePower);
    }

    public double getSlidePosition(Sides side) {
        double position = 0;
        if (side == Sides.LEFT) {
            position = leftSlide.getCurrentPosition();
        } else if (side == Sides.RIGHT) {
            position = rightSlide.getCurrentPosition();
        }
        return position;
    }

    public double getHandPosition() {
        double position = hand.getCurrentPosition();
        return position;
    }

    public void closeTrapdoor() {
        trapdoor.setPosition(0.5);
        trapdoorStatus = trapdoorPositions.CLOSE;
    }
    public void openTrapdoor() {
        trapdoor.setPosition(Servo.MIN_POSITION);
        trapdoorStatus = trapdoorPositions.OPEN;
    }
}