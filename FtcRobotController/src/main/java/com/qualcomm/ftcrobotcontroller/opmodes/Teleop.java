package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Robotics  on 10/28/2015.
 */

public class Teleop extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor tapeMotor1;
    DcMotor tapeMotor2;
    Servo bucket;
    Servo armRelease1;
    Servo armRelease2;
    //Servo arm;

    MediaPlayer mp = new MediaPlayer();

    private static final String frontLeft =  "front_left";
    private static final String frontRight = "front_right";
    private static final String tape1Name = "tape_right";
    private static final String tape2Name = "tape_left";
    private static final String bucketName = "bucket";
    private static final String release1Name = "dropit1";
    private static final String release2Name = "dropit2";

    private static final double frontMotorMultiple = 1.0;
    private static final double triggerThresh = 0.05;

    private static final double bucketPowerUp = 0;
    private static final double bucketPowerDown = 0.90;

    private static final double releasePowerUp = 0.55;
    private static final double releasePowerDown = 0.0;

    private double servoVal = 0.0;
    //private static final double armUpPosition = 0.0;
    //private static final double armLeftPosition = -1.0;
    //private static final double armRightPosition = 1.0;

    //constructor
    public Teleop() {
        try{
            mp.setDataSource("/storage/emulated/0/JOHNCENA.mp3");//Write your location here
            mp.prepare();
        }catch(Exception e){ e.printStackTrace(); }

    }

    //copy pasted from k9TeleOP
    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get(frontLeft);
        frontRightMotor = hardwareMap.dcMotor.get(frontRight);
        tapeMotor1 = hardwareMap.dcMotor.get(tape1Name);
        tapeMotor2 = hardwareMap.dcMotor.get(tape2Name);
        bucket = hardwareMap.servo.get(bucketName);
        armRelease1 = hardwareMap.servo.get(release1Name);
        armRelease2 = hardwareMap.servo.get(release2Name);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        tapeMotor1.setDirection(DcMotor.Direction.REVERSE);
        bucket.setPosition(bucketPowerDown);
        //arm.setPosition(armUpPosition);
        armRelease1.setDirection(Servo.Direction.FORWARD);
        armRelease2.setDirection(Servo.Direction.REVERSE);

        armRelease1.setPosition(releasePowerDown);
        armRelease2.setPosition(releasePowerDown);
    }

    @Override
    public void start(){
        mp.start();
    }

    @Override
    public void loop() {

        float leftThrottle = gamepad1.left_stick_y;
        float rightThrottle = gamepad1.right_stick_y;

        // write the values to the motors
        frontLeftMotor.setPower(leftThrottle * frontMotorMultiple);
        frontRightMotor.setPower(rightThrottle * frontMotorMultiple);

        // update the position of everything else

        if(gamepad1.right_trigger > triggerThresh) runTape(gamepad1.right_trigger);
        else if(gamepad1.left_trigger > triggerThresh) runTape(-gamepad1.left_trigger);
        else runTape(0);

        if(gamepad1.left_bumper) armRelease1.setPosition(releasePowerUp);
        else armRelease1.setPosition(releasePowerDown);
        //if(gamepad1.dpad_up) servoVal += 0.01;
        //else if(gamepad1.dpad_down) servoVal -= 0.01;
        //servoVal = Range.clip(servoVal, 0.0, 1.0);
        //armRelease1.setPosition(servoVal);
        //telemetry.addData("Servo: ", servoVal);

        if(gamepad1.right_bumper) armRelease2.setPosition(releasePowerUp);
        else armRelease2.setPosition(releasePowerDown);

        if(gamepad1.a) bucket.setPosition(bucketPowerUp);
        else bucket.setPosition(bucketPowerDown);

        //if(gamepad1.dpad_up) arm.setPosition(armUpPosition);
        //else if(gamepad1.dpad_left) arm.setPosition(armLeftPosition);
        //else if(gamepad1.dpad_right) arm.setPosition(armRightPosition);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** F***YEAH!!!***");
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", leftThrottle));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", rightThrottle));
    }

    @Override
    public void stop() {
        telemetry.addData("Text", "****ROBOT IS STOPPED****");
        mp.stop();
    }

    private void runTape(double power){
        tapeMotor2.setPower(power);
        tapeMotor1.setPower(power);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
