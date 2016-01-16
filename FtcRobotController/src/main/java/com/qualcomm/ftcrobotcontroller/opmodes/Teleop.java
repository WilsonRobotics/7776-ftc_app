package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics  on 10/28/2015.
 */

public class Teleop extends OpMode {
    DcMotor frontLeftMotor; //motor declarations, actual motor names will be later on
    DcMotor frontRightMotor;
    DcMotor tapeMotor1;
    Servo bucket;
    //Servo leftArm;
    //Servo leftClaw;
    //Servo rightClaw;

    MediaPlayer mp=new MediaPlayer();
    //boolean servosWork = true;
    boolean passive = false;

    private static final String frontLeft =  "front_left";  //motor name defines
    private static final String frontRight = "front_right";
    //private static final String backLeft = "back_left";
    //private static final String backRight = "back_right";
    private static final String tape1Name = "tape";
    //private static final String tape2Name = "tape2";
    //private static final String leftClawName = "left_claw";
    //private static final String rightClawName = "right_claw";
    private static final String bucketName = "bucket";
    //private static final String sweepName = "sweep";

    private static final double frontMotorMultiple = 1.0;
    //private static final double backMotorMultiple = 1.0;
    private static final double triggerThresh = 0.05;

    private static final double hookPowerUp = 1;
    private static final double hookPowerDown = -1;

    private static final double bucketPowerUp = 0;
    private static final double bucketPowerDown = 0.75;

    private static final double sweepPowerUp = 1.0;
    private static final double sweepPowerDown = -1.0;

    private static final double left_servo_idle = 0.5;
    private static final double right_servo_idle = 0.5;
    private static final double left_servo_lower = 1.0;
    private static final double right_servo_lower = 1.0;

    //constructor
    public Teleop() {
        try{
            mp.setDataSource("/storage/emulated/0/JOHNCENA.mp3");//Write your location here
            mp.prepare();
        }catch(Exception e){e.printStackTrace();}

    }

    //copy pasted from k9TeleOP
    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get(frontLeft);
        frontRightMotor = hardwareMap.dcMotor.get(frontRight);
        tapeMotor1 = hardwareMap.dcMotor.get(tape1Name);
        bucket = hardwareMap.servo.get(bucketName);
        //try{
        //    leftClaw = hardwareMap.servo.get(leftClawName);
        //    rightClaw = hardwareMap.servo.get(rightClawName);
        //}
        //catch (Exception ex){
        //    telemetry.addData("Servos Disconnected", 0);
        //    servosWork=false;
        //
        //}


        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        bucket.setPosition(bucketPowerDown);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        tapeMotor1.setDirection(DcMotor.Direction.REVERSE);
        //arm = hardwareMap.servo.get("servo_1");
        //claw = hardwareMap.servo.get("servo_6");

        // set the gamepad 2 dead zone to 0
        gamepad2.setJoystickDeadzone(0.0f);

    }

    @Override
    public void start(){
        mp.start();
    }

    @Override
    public void loop() {

		/*
		Gamepad 1 controls the motors via the left stick, and it controls the
		wrist/claw via the a,b, x, y buttons

        throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        1 is full down
        direction: left_stick_x ranges from -1 to 1, where -1 is full left
        and 1 is full right
        */

        /*
        TANK STEERING MOTHERFUCKAH
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        */

        float leftThrottle = gamepad1.left_stick_y;
        float rightThrottle = gamepad1.right_stick_y;

        // write the values to the motors
        frontLeftMotor.setPower(leftThrottle * frontMotorMultiple);
        frontRightMotor.setPower(rightThrottle * frontMotorMultiple);
        // update the position of everything else

        if(gamepad1.x) passive=true;
        else if(gamepad1.y) passive=false;

        if (gamepad1.right_trigger > triggerThresh) runTape(gamepad1.right_trigger);
        else if(gamepad1.left_trigger > triggerThresh) runTape(-gamepad1.left_trigger);
        else if(passive) runTape(0.2);
        else runTape(0);

        if(gamepad1.a) bucket.setPosition(bucketPowerUp);
        else bucket.setPosition(bucketPowerDown);



        //if(servosWork){
        //    if(gamepad1.left_bumper) leftClaw.setPosition(left_servo_lower);
        //    else leftClaw.setPosition(left_servo_idle);
        //
        // if(gamepad1.right_bumper) rightClaw.setPosition(right_servo_lower);
        //    else rightClaw.setPosition(right_servo_idle);
        //          }

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** F***YEAH!!!***");
        //telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", leftThrottle));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", rightThrottle));
    }

    @Override
    public void stop() {
        telemetry.addData("Text", "****ROBOT IS STOPPED****");
        mp.stop();
    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    private void runTape(double power){
        tapeMotor1.setPower(power);
    }

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
