package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Robotics on 12/2/2015.
 */
public class StateMachineAuto extends OpMode {

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    private static final String front_left_name = "front_left";
    private static final String front_right_name = "front_right";
    private static final String back_left_name = "back_left";
    private static final String back_right_name = "back_right";

    private final pathPart driveBox[] = {
            new pathPart(360, 360, 1, 1),
            new pathPart(360, -360, 1, -1),
            new pathPart(1800, 1800, 1, 1)
    };

    State utterState;
    path superPath;
    boolean firstCall=true;

    @Override
    public void init() {
        //specify configuration name save from scan operation
        front_left = hardwareMap.dcMotor.get(front_left_name);
        front_right = hardwareMap.dcMotor.get(front_right_name);
        back_left = hardwareMap.dcMotor.get(back_left_name);
        back_right = hardwareMap.dcMotor.get(back_right_name);

        //set servo positions later

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

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
                    //
                break;
            case DrivingToWall:

                break;
            case DropClimbers:

                break;
            case DetectColorSec:

                break;
            case DrivingUpMountain:

                break;
            case PlayingSound:

                break;
            case End:

                break;
        }
        telemetry.addData("State: ", utterState.getState());
    }

    @Override
    public void stop(){
        setDrivePower(0,0);
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
                    setEncoderTarget(0,0);
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
        DrivingUpMountain,
        PlayingSound,
        End,
        Null
    }

    public class State{
        private someStates currentState;

        public someStates getState(){
            return currentState;
        }

        public void setState(someStates setstate) {
            currentState = setstate;
        }
    }



}
