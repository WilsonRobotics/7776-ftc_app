package com.qualcomm.ftcrobotcontroller.opmodes;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */


public class RealTalkAuto extends OpMode {

    AutoLib.Sequence mainSequence;
    boolean bDone;                  // true when the programmed sequence is done

    DcMotor front_left;
    DcMotor front_right;
    GyroSensor gyro;
    Servo bucket;
    MediaPlayer mp;

    boolean red = true;

    boolean red = true;

    private static final String front_left_name = "front_left";
    private static final String front_right_name = "front_right";
    private static final String back_left_name = "back_left";
    private static final String back_right_name = "back_right";
    private static final String tape_motor_name = "tape";
    private static final String bucket_name = "bucket";
    private static final String gyro_name = "gyro";
    private static final String color_sensor_name = "color";
    private static final String left_claw_name = "left_claw";
    private static final String right_claw_name = "right_claw";

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

    public RealTalkAuto() {
    }

    public void init() {
        front_left = hardwareMap.dcMotor.get(front_left_name);
        front_right = hardwareMap.dcMotor.get(front_right_name);
        bucket = hardwareMap.servo.get(bucket_name);
        gyro = hardwareMap.gyroSensor.get(gyro_name);
        gyro.calibrate();

        front_right.setDirection(DcMotor.Direction.REVERSE);

        mp = new MediaPlayer();
        mp.setVolume(1.0f, 1.0f);

        mainSequence = new AutoLib.LinearSequence();

        bucket.setPosition(bucketPowerDown);

        /*
        mainSequence.add(new AutoLib.LogTimeStep(this, "Waiting 11 Seconds", 11));

        AutoLib.LinearSequence drivingToBox = new AutoLib.LinearSequence();
        drivingToBox.add(new AutoLib.MoveByEncoder(front_right, null, front_left, null,
                forward_power, 4500, true));
        if(!red) drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                -forward_power, forward_power, 1300, 1300, true));
        else drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                forward_power, -forward_power, 1300, 1300, true));
        drivingToBox.add(new AutoLib.MoveByEncoder(front_right, null, front_left, null,
                forward_power, 10000, true));
        if(!red) drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                -forward_power, forward_power, 1200, 1200, true));
        else drivingToBox.add(new AutoLib.TurnByEncoder(front_right, null, front_left, null,
                -forward_power, forward_power, 1200, 1200, true));
        drivingToBox.add(new AutoLib.MoveByEncoder(front_right, null, front_left, null,
                forward_power/2, 3000, true));
        mainSequence.add(drivingToBox);

        */

        mainSequence.add(new AutoLib.TurnByGyro(front_right, null, front_left, null, gyro, 0.5, -0.5, 45, 0.01, true));

        AutoLib.LinearSequence dumpingClimber = new AutoLib.LinearSequence();
        dumpingClimber.add(new AutoLib.TimedServoStep(bucket, bucketPowerUp, 2, false));
        dumpingClimber.add(new AutoLib.TimedSongStep(mp, "/storage/emulated/0/BUCKETS.wav", 2));
        dumpingClimber.add(new AutoLib.TimedServoStep(bucket, bucketPowerDown, 1, false));
        mainSequence.add(dumpingClimber);

        AutoLib.LinearSequence playingAwesome = new AutoLib.LinearSequence();
        playingAwesome.add(new AutoLib.TimedSongStep(mp, "/storage/emulated/0/JOHNCENA.mp3", 10000));

        // start out not-done
        bDone = false;

    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mainSequence.loop();       // returns true when we're done
    }

    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        telemetry.addData("stop() called", "");
    }
}


/* for reference
        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a first simple Step to the root Sequence
        mSequence.add(new AutoLib.LogTimeStep(this, "step1", 10));

        // create a ConcurrentSequence with 3 concurrent Steps
        AutoLib.ConcurrentSequence cs1 = new AutoLib.ConcurrentSequence();
            // step 1 of the 3 concurrent steps
            cs1.add(new AutoLib.LogTimeStep(this, "step2a", 10));
            // step 2 is itself a LinearSequence of two Steps
            AutoLib.LinearSequence cs1a = new AutoLib.LinearSequence();
                cs1a.add(new AutoLib.LogTimeStep(this, "step2b1", 6));
                cs1a.add(new AutoLib.LogTimeStep(this, "step2b2", 9));
            cs1.add(cs1a);
            // step 3 is a simple Step
            cs1.add(new AutoLib.LogTimeStep(this, "step2c", 5));
        // add the ConcurrentSequence to the root Sequence
        mSequence.add(cs1);

        // finish up with another simple Step
        mSequence.add(new AutoLib.LogTimeStep(this, "step3", 10));
        */