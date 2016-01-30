package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.ModernRoboticsI2cGyro;


/**
 * Created by phanau on 1/22/16.
 * Test hardware gyro
 */
public class GyroTestOp extends OpMode {

    private GyroSensor mGyro;

    public GyroTestOp() {
    }

    public void init() {
        // get hardware gyro and start calibrating it
        mGyro = (GyroSensor) hardwareMap.gyroSensor.get("gyro");
        mGyro.calibrate();      // start calibration -- wait for it to complete in loop() function
    }

    public void loop() {
        telemetry.addData("status:  ", mGyro.isCalibrating() ? "calibrating" : "ready");
        telemetry.addData("heading: ", mGyro.isCalibrating() ? "calibrating" : mGyro.getHeading());
        if (mGyro instanceof ModernRoboticsI2cGyro) {
            telemetry.addData("integrated z: ", mGyro.isCalibrating() ? "calibrating" : ((ModernRoboticsI2cGyro)mGyro).getIntegratedZValue());
        }
    }

    public void stop() {
        mGyro.close();
    }

}
