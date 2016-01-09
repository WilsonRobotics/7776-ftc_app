package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * simple test OpMode that reads and logs sensors
 * Created by phanau on 1/1/16.
 */

public class SensorFilterTestOp extends OpMode {

    private SensorLib.FilteredDirectionSensor mDirSensor;

    public SensorFilterTestOp() {
        mDirSensor = new SensorLib.FilteredDirectionSensor();
    }

    public void init() {
        mDirSensor.init();
    }

    public void loop() {
        telemetry.addData("updates: ", mDirSensor.dataCount());
        telemetry.addData("azimuth: ", mDirSensor.azimuth());
        telemetry.addData("pitch:   ", mDirSensor.pitch());
        telemetry.addData("roll:    ", mDirSensor.roll());
    }

    public void stop() {
        mDirSensor.stop();
    }
}
