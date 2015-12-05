package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Util;

/**
 * Created by Robotics on 12/2/2015.
 */
public class StateMachineAuto extends OpMode {

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    ColorSensor color;

    private static final String front_left_name = "front_left";
    private static final String front_right_name = "front_right";
    private static final String back_left_name = "back_left";
    private static final String back_right_name = "back_right";
    private static final String color_sensor_name = "color";

    private static final double mountain_idle_power = 0.1;
    private static final int red_thresh = 128;
    private static final int blue_thresh = 128;
    private static final int moving_avg_amount = 20;

    private final pathPart driveBox[] = {
            new pathPart(360, 360, 1, 1), //forward
            new pathPart(360, -360, 1, -1), //turn left
            new pathPart(1800, 1800, 1, 1) //forward
    };

    private final pathPart driveWall[] = {
            new pathPart(720, 720, 1, 1) //SMASH!
    };

    private final pathPart driveToMountain[] = {
            new pathPart(-720, -720, -1, -1), //backward
            new pathPart(720, -720, 1, -1), //turn left a ton
            new pathPart(1000, 1000, 1, 1), //FORWARD!
            new pathPart(-720, 720, -1, 1), //turn right towards mountain
            new pathPart(720, 720, 1, 1) //get REALLY close
    };

    private final pathPart driveForward[] = {
            new pathPart(720, 720, 1, 1)
    };

    private final pathPart driveUpMountain[] = {
            new pathPart(1000, 1000, 1, 1)
    };
    State utterState;
    path superPath;
    colorSens color_sensor = new colorSens(true); //true for red
    boolean firstCall=true;

    @Override
    public void init() {
        //specify configuration name save from scan operation
        front_left = hardwareMap.dcMotor.get(front_left_name);
        front_right = hardwareMap.dcMotor.get(front_right_name);
        back_left = hardwareMap.dcMotor.get(back_left_name);
        back_right = hardwareMap.dcMotor.get(back_right_name);
        color = hardwareMap.colorSensor.get(color_sensor_name);

        //set servo positions later

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        color.enableLed(true);

        setDrivePower(0, 0);
    }

    @Override
    public void init_loop(){
        resetEncoders();
    }

    @Override
    public void start() {
        utterState.setState(someStates.Idle);
        runToPosition();
        setEncoderTarget(0, 0);
    }


    @Override
    public void loop() {
        switch (utterState.getState()){
            case Idle:
                resetEncoders();
                if(getLeftEncoders() == 0 && getRightEncoders() == 0) utterState.setState(someStates.DrivingToBox);
                else{
                    resetEncoders();
                    telemetry.addData("Encoders Not Zero!" ,String.format("L:R %d:%d", getLeftEncoders(), getRightEncoders()));
                }
                break;
            case DrivingToBox:
                if(firstCall){
                    superPath.begin(driveBox);
                    superPath.update();
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState.setState(someStates.DetectColorFirst);
                    firstCall=true;
                }
                break;
            case DetectColorFirst:
                if(firstCall){
                    color_sensor.reset();
                    runNormal();
                    setDrivePower(0.2, 0.2);
                    for(int i=0; i<moving_avg_amount; i++){
                        color_sensor.checkColor();
                    }
                    firstCall=false;
                }
                else if(color_sensor.checkColor()){
                    setDrivePower(0,0);
                    firstCall=false;
                    utterState.setState(someStates.DrivingToWall);
                }
                break;
            case DrivingToWall:
                if(firstCall){
                    superPath.begin(driveWall);
                    superPath.update();
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState.setState(someStates.DropClimbers);
                    firstCall=true;
                }
                break;
            case DropClimbers:
                //servo stuff TODO
                utterState.setState(someStates.DetectColorSec);
                break;
            case DetectColorSec:
                if(firstCall){
                    color_sensor.reset();
                    runNormal();
                    setDrivePower(-0.2, -0.2);
                    for(int i=0; i<moving_avg_amount; i++){
                        color_sensor.checkColor();
                    }
                    firstCall=false;
                }
                else if(color_sensor.checkColor()){
                    setDrivePower(0,0);
                    firstCall=false;
                    utterState.setState(someStates.DrivingUpMountain);
                }
                break;
            case DrivingTowardsMountain:
                if(firstCall){
                    superPath.begin(driveToMountain);
                    superPath.update();
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState.setState(someStates.KnockClimber);
                    firstCall=true;
                }
                break;
            case KnockClimber:
                //Servo code TODO
                if(firstCall){
                    superPath.begin(driveForward);
                    superPath.update();
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState.setState(someStates.DrivingUpMountain);
                    firstCall=true;
                }
                break;
            case DrivingUpMountain:
                if(firstCall){
                    superPath.begin(driveUpMountain);
                    superPath.update();
                    firstCall=false;
                }
                else if(!superPath.update()){
                    utterState.setState(someStates.PlayingSound);
                    firstCall=true;
                }
                break;
            case PlayingSound:
                //TODO: Sound code
                utterState.setState(someStates.End);
                break;
            case End:
                runNormal();
                setDrivePower(mountain_idle_power, mountain_idle_power);
                telemetry.addData("ALL DONE!", 0);
                break;
        }
        telemetry.addData("State: ", utterState.getState());
    }

    @Override
    public void stop(){
        runNormal();
        setDrivePower(0, 0);
    }

    void setDrivePower(double left, double right){
        front_left.setPower(left);
        front_right.setPower(right);
        back_left.setPower(left);
        back_right.setPower(right);
    }

    void resetEncoders(){
        front_left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        front_right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        back_left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        back_right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void runToPosition(){
        front_left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        front_right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        back_left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        back_right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    void runNormal(){
        front_left.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        front_right.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        back_left.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        back_right.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    int getLeftEncoders(){
        return Math.abs((front_left.getCurrentPosition() + back_left.getCurrentPosition()) / 2);
    }

    int getRightEncoders(){
        return Math.abs((front_right.getCurrentPosition()+back_right.getCurrentPosition())/2);
    }

    void setEncoderTarget(int left, int right){
        front_left.setTargetPosition(left);
        front_right.setTargetPosition(right);
        back_left.setTargetPosition(left);
        back_right.setTargetPosition(right);
    }

    private class pathPart{
        public double leftPower;
        public double rightPower;
        public int leftDistance;
        public int rightDistance;

        public pathPart(int leftDis, int rightDis, double leftPow, double rightPow){
            leftPower=leftPow;
            rightPower=rightPow;
            leftDistance=leftDis;
            rightDistance=rightDis;
        }
    }

    private class path{
        boolean motorsSet;
        boolean pathSet;
        private pathPart currentPath[];
        private int currentSeg;


        public void begin(pathPart runPath[]){
            currentPath=runPath;
            currentSeg=0;
        }

        public boolean update(){
            if(currentSeg!=-1){
                if(getLeftEncoders() > currentPath[currentSeg].leftDistance && getRightEncoders() > currentPath[currentSeg].rightDistance){
                    motorsSet=false;
                    setEncoderTarget(0, 0);
                    setDrivePower(0, 0);
                    resetEncoders();
                    currentSeg++;
                    if(currentSeg>currentPath.length) {
                        currentSeg = -1;
                    }
                }
                else if(!motorsSet) {
                    runToPosition();
                    setEncoderTarget(currentPath[currentSeg].leftDistance, currentPath[currentSeg].rightDistance);
                    setDrivePower(currentPath[currentSeg].leftPower, currentPath[currentSeg].rightPower);
                }
                return true;
            }
            return false;
        }
    }

    public enum someStates{
        Idle,
        DrivingToBox,
        DetectColorFirst,
        DrivingToWall,
        DropClimbers,
        DetectColorSec,
        DrivingTowardsMountain,
        KnockClimber,
        DrivingUpMountain,
        PlayingSound,
        End,
        Null
    }

    private class State{
        private someStates currentState;

        public someStates getState(){
            return currentState;
        }

        public void setState(someStates setstate) {
            currentState = setstate;
        }
    }

    private class movingAvg{
        private int[] moving = new int[moving_avg_amount];

        public boolean checkThreshold(int input, int thresh){
            for(int i=0; i<moving_avg_amount-1; i++){
                moving[i] = moving[i+1];
            }
            moving[moving_avg_amount-1] = input;
            double avg = 0;
            for(int i=0; i<moving_avg_amount; i++){
                avg+=moving[i];
            }
            avg/=moving_avg_amount;
            return thresh>avg;
        }

        public void reset(){
            moving = new int[10];
        }
    }

    private class colorSens{
        private movingAvg ret;
        private boolean redOrBlue;

        colorSens(boolean red){
            redOrBlue=red;
            ret.reset();
        }
        boolean checkColor(){
            if(redOrBlue){
                return ret.checkThreshold(color.red(), red_thresh);
            }
            else{
                return ret.checkThreshold(color.blue(), blue_thresh);
            }
        }

        public void reset(){
            ret.reset();
        }
    }


}