package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Robotics on 12/2/2015.
 */

public class RedAuto extends OpMode {
    private static final boolean testState = false;
    private static final someStates testingState = someStates.DrivingToMountain;
    private boolean servosWork = true;

    MediaPlayer mp=new MediaPlayer();

    private static final String front_left_name = "front_left";
    private static final String front_right_name = "front_right";
    private static final String back_left_name = "back_left";
    private static final String back_right_name = "back_right";
    private static final String color_sensor_name = "color";
    private static final String left_claw_name = "left_claw";
    private static final String right_claw_name = "right_claw";

    private static final double mountain_idle_power = 0.1;
    private static final int red_thresh = 128;
    private static final int blue_thresh = 128;
    private static final int moving_avg_amount = 20;
    private static final double left_servo_idle = 0.5;
    private static final double right_servo_idle = 0.5;
    private static final double left_servo_lower = 1.0;
    private static final double right_servo_lower = 1.0;

    public enum someStates {
        Idle,
        Wait10,
        DrivingToMountain,
        LoweringTrigger,
        DrivingUpMountain,
        IdleMountain,
        PlayingSound,
        End,
        Null
    }

    private final pathPart driveToMountain[] = {
            new pathPart(1000, 1000, 1, 1), //forward
            new pathPart(-360, 360, -1, 1), //right
            new pathPart(1000, 1000, 1, 1), //forward
            new pathPart(-360, 360, -1, 1), //right
            new pathPart(360, 360, 1, 1) //forward
    };

    private final pathPart driveUpMountain[] = {
            new pathPart(1000, 1000, 1, 1)
    };

    // GYRO STUFF
    public static final int GYRO_ADDRESS = 0x68;

    // These byte values are common with most Modern Robotics sensors.
    public static final int READ_MODE = 0x80;
    public static final int ADDRESS_MEMORY_START = 0x0;
    public static final int TOTAL_MEMORY_LENGTH = 0x0c;
    public static final int BUFFER_CHANGE_ADDRESS_LENGTH = 0x03;

    int gyro_port = 5;

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    //DeviceInterfaceModule dim;

    ElapsedTime time;
    DcMotor front_left;
    DcMotor front_right;
    //DcMotor back_left;
    //DcMotor back_right;
    Servo leftClaw;
    Servo rightClaw;

    someStates utterState;
    path superPath;
    boolean firstCall = true;


    @Override
    public void init() {
        superPath = new path();
        //specify configuration name save from scan operation
        front_left = hardwareMap.dcMotor.get(front_left_name);
        front_right = hardwareMap.dcMotor.get(front_right_name);
        //back_left = hardwareMap.dcMotor.get(back_left_name);
        //back_right = hardwareMap.dcMotor.get(back_right_name);

        //set servo positions later
        try{
            leftClaw = hardwareMap.servo.get(left_claw_name);
            rightClaw = hardwareMap.servo.get(right_claw_name);
        }
        catch(Exception ex){
            telemetry.addData("Servos not connected", 0);
            servosWork=false;
        }
        front_right.setDirection(DcMotor.Direction.REVERSE);
        //back_right.setDirection(DcMotor.Direction.REVERSE);

        if(servosWork){
            leftClaw.setPosition(left_servo_idle);
            rightClaw.setPosition(right_servo_idle);
        }
        setDrivePower(0, 0);

        try{
            mp.setDataSource("/storage/emulated/0/theme.mp3");//Write your location here
            mp.prepare();
        }catch(Exception e){e.printStackTrace();}

        //dim = hardwareMap.deviceInterfaceModule.get("dim");

        //readCache = dim.getI2cReadCache(gyro_port);
        //writeCache = dim.getI2cWriteCache(gyro_port);

        // wake up the gyro
        //performAction("write", gyro_port, GYRO_ADDRESS, );
    }

    @Override
    void stop_motors(){
        run_motors(0, 0);
        resetEncoders();
    }

    void run_motors_until(int lPower, int rPower, int distance){
        int lDistance, rDistance;
        if(lPower > 0) lDistance = distance;
        else lDistance = -distance;
        
        if(rPower > 0) rDistance = distance;
        else rDistance = -distance;
        runToPosition(lDistance, rDistance);
        run_motors(lPower, rPower);
    }

    void bSleep(long time){
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
    }


    @Override
    public void loop() {
        switch (utterState) {
            case Idle:
                if (getLeftEncoders() == 0 && getRightEncoders() == 0)
                    utterState = someStates.DrivingToMountain;
                else {
                    resetEncoders();
                    telemetry.addData("Encoders Not Zero!", String.format("L:R %d:%d", getLeftEncoders(), getRightEncoders()));
                }
                break;
            case DrivingToMountain:
                if(firstCall){
                    superPath.begin(driveToMountain);
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState = someStates.LoweringTrigger;
                    firstCall=true;
                }
                break;
            case LoweringTrigger:
                if(servosWork){
                    leftClaw.setPosition(left_servo_lower);
                    rightClaw.setPosition(right_servo_lower);
                }
                utterState = someStates.DrivingUpMountain;
                break;
            case DrivingUpMountain:
                if (firstCall) {
                    superPath.begin(driveUpMountain);
                    firstCall = false;
                } else if (!superPath.update()) {
                    utterState = someStates.PlayingSound;
                    firstCall = true;
                }
                break;
            case IdleMountain:
                runNormal();
                setDrivePower(mountain_idle_power, mountain_idle_power);
                utterState = someStates.PlayingSound;
                break;
            case PlayingSound:
                mp.start();
                utterState = someStates.End;
                break;
            case End:
                telemetry.addData("ALL DONE!", 0);
                break;
        }
        telemetry.addData("State: ", utterState);
        if (testState && utterState != testingState) utterState = someStates.End;
    }

    @Override
    public void stop() {
        runNormal();
        setDrivePower(0, 0);
    }

    void setDrivePower(double left, double right) {
        front_left.setPower(left);
        front_right.setPower(right);
        //back_left.setPower(left);
        //back_right.setPower(right);
    }

    void resetEncoders() {
        front_left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        front_right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //back_left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //back_right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void runToPosition() {
        front_left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //back_left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //back_right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    void runNormal() {
        front_left.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        front_right.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //back_left.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //back_right.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    int getLeftEncoders() {
        return Math.abs((front_left.getCurrentPosition())); //+ back_left.getCurrentPosition()) / 2);
    }

    int getRightEncoders() {
        return Math.abs((front_right.getCurrentPosition())); //+back_right.getCurrentPosition())/2);
    }

    void setEncoderTarget(int left, int right) {
        front_left.setTargetPosition(left);
        front_right.setTargetPosition(right);
        //back_left.setTargetPosition(left);
        //back_right.setTargetPosition(right);
    }

    private class pathPart {
        public double leftPower;
        public double rightPower;
        public int leftDistance;
        public int rightDistance;

        public pathPart(int leftDis, int rightDis, double leftPow, double rightPow) {
            leftPower = leftPow;
            rightPower = rightPow;
            leftDistance = leftDis;
            rightDistance = rightDis;
        }
    }

    private class path {
        boolean pathSet;
        private pathPart currentPath[];
        private int currentSeg = -1;
        boolean motorsSet = false;


        public void begin(pathPart runPath[]) {
            currentPath = runPath;
            currentSeg = 0;
            telemetry.addData("Begining Path", 0);
            runToPosition();
            setEncoderTarget(currentPath[currentSeg].leftDistance, currentPath[currentSeg].rightDistance);
            setDrivePower(currentPath[currentSeg].leftPower, currentPath[currentSeg].rightPower);
            telemetry.addData("Motors Set!", currentPath[currentSeg].leftDistance);
        }

        public boolean update() {
            if (currentSeg != -1) {
                if (getLeftEncoders() > currentPath[currentSeg].leftDistance && getRightEncoders() > currentPath[currentSeg].rightDistance) {
                    setEncoderTarget(0, 0);
                    setDrivePower(0, 0);
                    resetEncoders();
                    currentSeg++;
                    motorsSet=false;
                    if (currentSeg >= currentPath.length) {
                        currentSeg = -1;
                    }
                }
                else if(!motorsSet) {
                    runToPosition();
                    setEncoderTarget(currentPath[currentSeg].leftDistance, currentPath[currentSeg].rightDistance);
                    setDrivePower(currentPath[currentSeg].leftPower, currentPath[currentSeg].rightPower);
                    telemetry.addData("Motors Set!", currentPath[currentSeg].leftDistance);
                    motorsSet=true;
                }
                return true;
            }
            return false;
        }
    }
