package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.IOException;

/**
 * Created by Robotics on 10/28/2015.
 */

public class RatBot extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;

    MediaPlayer mp;

    @Override
    public void init() {
        //media stuff
        mp = new MediaPlayer();
        try {
            mp.setDataSource("/storage/emulated/0/AXELF.mp3");
        } catch (IOException e) {
            e.printStackTrace();
        }

        // hardware maps
        frontLeftMotor = hardwareMap.dcMotor.get("front_left");
        frontRightMotor = hardwareMap.dcMotor.get("front_right");

        // change directions if necessary
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {
        mp.start();
    }

    @Override
    public void loop() {

        // run the drive train motors
        frontLeftMotor.setPower(gamepad1.left_stick_y);
        frontRightMotor.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void stop() {
        telemetry.addData("Text", "****ROBOT IS STOPPED****");
        mp.stop();
    }
}

