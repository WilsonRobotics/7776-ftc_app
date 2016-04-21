package com.qualcomm.ftcrobotcontroller.opmodes;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */


public class RealTalkAutoRed extends OpMode {

    AutoLib.Sequence mainSequence;
    boolean bDone;                  // true when the programmed sequence is done

    DcMotor front_left;
    DcMotor front_right;
    Servo bucket;
    MediaPlayer mp;
    MediaPlayer emp;
    Servo armRelease1;
    Servo armRelease2;
    Servo flag1;
    Servo flag2;
    navXGyro mGyro;
    SensorLib.PID mPid;                     // PID controller for the sequence

    boolean red = true;

    private static final String front_left_name = "front_left";
    private static final String front_right_name = "front_right";
    private static final String back_left_name = "back_left";
    private static final String back_right_name = "back_right";
    private static final String tape_motor_name = "tape";
    private static final String bucket_name = "bucket";
    private static final String gyro_name = "gyro1";
    private static final String color_sensor_name = "color";
    private static final String left_claw_name = "left_claw";
    private static final String right_claw_name = "right_claw";
    private static final String release1Name = "dropit1";
    private static final String release2Name = "dropit2";
    private static final String flag1Name = "flag_left";
    private static final String flag2Name = "flag_right";
    private static final String dimName = "dim";
    private static final int dimPort = 0;

    private static final double forward_power = 1.0;
    private static final double mountain_idle_power = 0.1;
    private static final int red_thresh = 128;
    private static final int blue_thresh = 128;
    private static final int moving_avg_amount = 20;
    private static final double left_servo_idle = 0.5;
    private static final double right_servo_idle = 0.5;
    private static final double left_servo_lower = 1.0;
    private static final double right_servo_lower = 1.0;
    private static final double bucketPowerUp = 0;
    private static final double bucketPowerDown = 1;
    private static final double flappyArmPowerUp = 0.55;
    private static final double flappyArmPowerDown = 0.0;
    private static final double flagPullPowerUp = 1.0;
    private static final double flagPullPowerDown = 0.0;

    // parameters of the PID controller for this sequence
    float Kp = 0.10f;        // motor power proportional term correction per degree of deviation
    float Ki = 0.08f;         // ... integrator term
    float Kd = 0;             // ... derivative term
    float KiCutoff = 0.0f;    // maximum angle error for which we update integrator


    public void init() {
        front_left = hardwareMap.dcMotor.get(front_left_name);
        front_right = hardwareMap.dcMotor.get(front_right_name);
        bucket = hardwareMap.servo.get(bucket_name);
        armRelease1 = hardwareMap.servo.get(release1Name);
        armRelease2 = hardwareMap.servo.get(release2Name);
        flag1 = hardwareMap.servo.get(flag1Name);
        flag2 = hardwareMap.servo.get(flag2Name);

        mGyro = new navXGyro(hardwareMap.deviceInterfaceModule.get(dimName), dimPort);

        front_right.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.FORWARD);

        armRelease1.setDirection(Servo.Direction.FORWARD);
        armRelease2.setDirection(Servo.Direction.REVERSE);

        flag1.setDirection(Servo.Direction.FORWARD);
        flag2.setDirection(Servo.Direction.REVERSE);

        armRelease1.setPosition(flappyArmPowerDown);
        armRelease2.setPosition(flappyArmPowerDown);
        flag1.setPosition(flagPullPowerDown);
        flag2.setPosition(flagPullPowerDown+0.04);
        bucket.setPosition(bucketPowerDown);

        mp = new MediaPlayer();
        emp = new MediaPlayer();

        // create a PID controller for the sequence
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        EncoderMotor[] motors = {new EncoderMotor(front_right), null, new EncoderMotor(front_left), null};

        mainSequence = new AutoLib.LinearSequence();

        mainSequence.add(new AutoLib.LogTimeStep(this, "Waiting 8 Seconds", 8));

        AutoLib.LinearSequence drivingToBox = new AutoLib.LinearSequence();
        //drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
        //        right_forward_power, left_forward_power, 4500, 4500, true));
        drivingToBox.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, motors, (float)forward_power, 4500, true));
        if(!red) drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                -forward_power, forward_power, 1300, 1300, true));
        else drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                forward_power, -forward_power, 1300, 1300, true));
        //drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
        //        right_forward_power, left_forward_power, 9500, 9500, true));
        drivingToBox.add(new AutoLib.AzimuthCountedDriveStep(this, -45, mGyro, mPid, motors, (float)forward_power, 8500, true));
        if(!red) drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                -forward_power, forward_power, 1300, 1300, true));
        else drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                forward_power, -forward_power, 1300, 1300, true));
        //drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
        //        right_forward_power/2, left_forward_power/2, 3000, 3000, true));
        drivingToBox.add(new AutoLib.AzimuthCountedDriveStep(this, -90, mGyro, mPid, motors, (float)forward_power/4, 4000, true));
        mainSequence.add(drivingToBox);

        AutoLib.ConcurrentSequence dropClimbers = new AutoLib.ConcurrentSequence();
        AutoLib.LinearSequence servoStep = new AutoLib.LinearSequence();
        servoStep.add(new AutoLib.TimedServoStep(bucket, bucketPowerUp, 1.0, false));
        servoStep.add(new AutoLib.TimedServoStep(bucket, bucketPowerDown, 1.0, false));
        dropClimbers.add(servoStep);
        dropClimbers.add(new AutoLib.TimedSongStep(mp, "/storage/emulated/0/BUCKETS.mp3", 1));
        //mainSequence.add(dropClimbers);

        AutoLib.LinearSequence playingAwesome = new AutoLib.LinearSequence();
        playingAwesome.add(new AutoLib.TimedSongStep(emp, "/storage/emulated/0/JOHNCENA.mp3", 10000));
        mainSequence.add(playingAwesome);

        // start out not-done
        bDone = false;
    }

    @Override
    public void start(){
        mGyro.calibrate();
    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mainSequence.loop();       // returns true when we're done
    }

    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        mGyro.stop();
        telemetry.addData("stop() called", "");
    }
}