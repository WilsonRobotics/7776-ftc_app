package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * simple test OpMode that reads and logs sensors
 * Created by phanau on 1/1/16.
 */

public class SensorTestOp extends OpMode {

    private SensorLib.DirectionSensor mDirSensor;

    public SensorTestOp() {
        mDirSensor = new SensorLib.DirectionSensor();
    }

    public void init() {
        mDirSensor.init();
    }

    public void loop() {
        telemetry.addData("status:  ", mDirSensor.dataReady() ? "ready" : "not ready");
        telemetry.addData("azimuth: ", mDirSensor.azimuth());
        telemetry.addData("pitch:   ", mDirSensor.pitch());
        telemetry.addData("roll:    ", mDirSensor.roll());
    }

    public void stop() {
        mDirSensor.stop();
    }
}
