package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 10/28/2015.
 */

public class Teleop extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor tapeMotor1;
    DcMotor tapeMotor2;
    Servo bucket;
    Servo leftWing;
    Servo rightWing;
    Servo leftFlagPull;
    Servo rightFlagPull;

    MediaPlayer bucketPlay = new MediaPlayer();
    MediaPlayer eaglePlay = new MediaPlayer();

    private static final String frontLeftName =  "front_left";
    private static final String frontRightName = "front_right";
    private static final String tape1Name = "tape_right";
    private static final String tape2Name = "tape_left";
    private static final String bucketName = "bucket";
    private static final String leftWingName = "dropit1";
    private static final String rightWingName = "dropit2";
    private static final String leftFlagPullName = "flag_left";
    private static final String rightFlagPullName = "flag_right";

    private static final double triggerThresh = 0.05;   // the minimum value that must be read to trigger the wings

    // servo target positions
    private static final double bucketPowerUp = 0;
    private static final double bucketPowerDown = 0.90;
    private static final double wingPowerUp = 0.55;
    private static final double wingPowerDown = 0.1;
    private static final double flagPullPowerUp = 0.0;
    private static final double flagPullPowerDown = 0.5;

    // keep track of the last state of controller buttons for toggles
    private boolean leftBumperLastVal = false;
    private boolean rightBumperLastVal = false;
    private boolean leftTriggerLastVal = false;
    private boolean rightTriggerLastVal = false;

    // initial servo positions
    private boolean wingLeftPos = false;
    private boolean wingRightPos = false;
    private boolean flagPullLeftPos = false;
    private boolean flagPullRightPos = false;

    // constructor
    public Teleop() {

        // setup the audio sources
        try {
            bucketPlay.setDataSource("/storage/emulated/0/BUCKETS.mp3");
            bucketPlay.prepare();
            eaglePlay.setDataSource("/storage/emulated/0/EAGLE.mp3");
            eaglePlay.prepare();
        } catch(Exception e){ e.printStackTrace(); }
    }

    @Override
    public void init() {

        // hardware maps
        frontLeftMotor = hardwareMap.dcMotor.get(frontLeftName);
        frontRightMotor = hardwareMap.dcMotor.get(frontRightName);
        tapeMotor1 = hardwareMap.dcMotor.get(tape1Name);
        tapeMotor2 = hardwareMap.dcMotor.get(tape2Name);
        bucket = hardwareMap.servo.get(bucketName);
        leftWing = hardwareMap.servo.get(leftWingName);
        rightWing = hardwareMap.servo.get(rightWingName);
        leftFlagPull = hardwareMap.servo.get(leftFlagPullName);
        rightFlagPull = hardwareMap.servo.get(rightFlagPullName);

        // change directions if necessary
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        tapeMotor1.setDirection(DcMotor.Direction.REVERSE);
        tapeMotor2.setDirection(DcMotor.Direction.FORWARD);
        leftWing.setDirection(Servo.Direction.FORWARD);
        rightWing.setDirection(Servo.Direction.REVERSE);

        // set initial servo positions
        leftWing.setPosition(wingPowerDown);
        rightWing.setPosition(wingPowerDown);
        bucket.setPosition(bucketPowerDown);
    }

    @Override
    public void start() {
        // nothing to put here...
    }

    @Override
    public void loop() {

        // run the drive train motors
        frontLeftMotor.setPower(gamepad1.left_stick_y);
        frontRightMotor.setPower(gamepad1.right_stick_y);

        // run the tape measure motors
        if(gamepad1.right_trigger > triggerThresh) runTape(gamepad1.right_trigger);
        else if(gamepad1.left_trigger > triggerThresh) runTape(-gamepad1.left_trigger);
        else runTape(0);

        // toggle wing position variables
        if(gamepad2.left_trigger > triggerThresh && !leftTriggerLastVal) {
            wingLeftPos = !wingLeftPos;
            if(wingLeftPos) eaglePlay.start();
        }
        if(gamepad2.right_trigger > triggerThresh && !rightTriggerLastVal) {
            wingRightPos = !wingRightPos;
            if(wingRightPos) eaglePlay.start();
        }

        // update wing positions
        if(wingLeftPos) leftWing.setPosition(wingPowerUp);
        else leftWing.setPosition(wingPowerDown);
        if(wingRightPos) rightWing.setPosition(wingPowerUp);
        else rightWing.setPosition(wingPowerDown);

        // toggle flag pull position variables
        if(gamepad2.left_bumper && !leftBumperLastVal) flagPullLeftPos = !flagPullLeftPos;
        if(gamepad2.right_bumper && !rightBumperLastVal) flagPullRightPos = !flagPullRightPos;

        // update flag pull positions
        if(flagPullLeftPos) leftFlagPull.setPosition(flagPullPowerUp);
        else leftFlagPull.setPosition(flagPullPowerDown);
        if(flagPullRightPos) rightFlagPull.setPosition(flagPullPowerUp);
        else rightFlagPull.setPosition(flagPullPowerDown);

        // record controller button states for toggles
        leftTriggerLastVal = gamepad2.left_trigger > triggerThresh;
        rightTriggerLastVal = gamepad2.right_trigger > triggerThresh;
        leftBumperLastVal = gamepad2.left_bumper;
        rightBumperLastVal = gamepad2.right_bumper;

        // update bucket position
        if(gamepad1.a) {
            bucket.setPosition(bucketPowerUp);
            bucketPlay.start();
        }
        else bucket.setPosition(bucketPowerDown);
    }

    @Override
    public void stop() {
        telemetry.addData("Text", "****ROBOT IS STOPPED****");
        bucketPlay.stop();
        eaglePlay.stop();
    }

    private void runTape(double power) {
        tapeMotor1.setPower(power);
        tapeMotor2.setPower(power);
    }
}
