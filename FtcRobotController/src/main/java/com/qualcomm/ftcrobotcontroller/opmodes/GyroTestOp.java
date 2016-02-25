package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


/**
 * Created by phanau on 1/22/16.
 * Test hardware gyro
 */
public class GyroTestOp extends OpMode {

    private ModernRoboticsI2cGyro mGyro1, mGyro2;
    private SensorLib.CorrectedMRGyro mCorrGyro;

    public GyroTestOp() {
    }

    public void init() {
        // get hardware gyro(s) -- we support either one or two, where the second is assumed
        // to be mounted in opposite z-orientation to the first.
        mGyro1 = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro1");
        try { mGyro2 = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro2"); }
        catch (Exception e) { mGyro2 = null; }

        // wrap gyro1 (and optional gyro2, if present) in an object that corrects its output
        mCorrGyro = new SensorLib.CorrectedMRGyro((ModernRoboticsI2cGyro) mGyro1, (ModernRoboticsI2cGyro) mGyro2);
        mCorrGyro.calibrate();      // calibrate the underlying hardware gyro
    }

    public void loop() {
        boolean calibrating = mGyro1.isCalibrating() || (mGyro2 != null && mGyro2.isCalibrating());
        telemetry.addData("status:  ", calibrating ? "calibrating" : "ready");
        telemetry.addData("integrated z1: ", mCorrGyro.getIntegratedZValue1());
        telemetry.addData("integrated z2: ", mCorrGyro.getIntegratedZValue2());
        telemetry.addData("corrected heading: ", mCorrGyro.getHeading());
    }

    public void stop() {
        mCorrGyro.stop();       // release the physical gyro(s) we're using
    }

}
