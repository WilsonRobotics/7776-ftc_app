package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Robotics on 12/11/2015.
 */
public class TestingClass extends OpMode {

    DcMotor front_left;
    DcMotor front_right;
    path testPath;

    pathPart[] test = {
            new pathPart(0,0,0,0),
            new pathPart(1,1,1,1)
    };

    @Override
    public void init(){
        front_left=hardwareMap.dcMotor.get("front_left");
        front_right=hardwareMap.dcMotor.get("front_right");
        testPath = new path();
        front_left.setDirection(DcMotor.Direction.REVERSE);
    }

    //@Override
    public void init_loop(){
        resetEncoders();
        telemetry.addData("Other Debug", test[1].leftPower);
    }

    @Override
    public void start(){
        runToPosition();
        testPath.begin(test);
    }

    @Override
    public void loop(){
        testPath.update();
    }

    @Override
    public void stop(){
        runNormal();
        setDrivePower(0,0);
    }

    void setDrivePower(double left, double right){
        front_left.setPower(left);
        front_right.setPower(right);
        //back_left.setPower(left);
        //back_right.setPower(right);
    }

    void resetEncoders(){
        front_left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        front_right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //back_left.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        //back_right.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void runToPosition(){
        front_left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //back_left.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //back_right.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    void runNormal(){
        front_left.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        front_right.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //back_left.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //back_right.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    int getLeftEncoders(){
        //return Math.abs((front_left.getCurrentPosition() + back_left.getCurrentPosition()) / 2);
        return Math.abs(front_left.getCurrentPosition());
    }

    int getRightEncoders(){
        //return Math.abs((front_right.getCurrentPosition()+back_right.getCurrentPosition())/2);
        return Math.abs(front_right.getCurrentPosition());
    }

    void setEncoderTarget(int left, int right){
        front_left.setTargetPosition(left);
        front_right.setTargetPosition(right);
        //back_left.setTargetPosition(left);
        //back_right.setTargetPosition(right);
    }

    class pathPart{
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

    public class path{
        boolean motorsSet = false;
        pathPart[] currentPath = {};
        int currentSeg = -1;


        public void begin(pathPart[] runPath){
            currentPath = runPath;
            currentSeg = 0;
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
                    motorsSet = true;
                }
                return true;
            }
            return false;
        }
    }
}

