package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import java.util.concurrent.locks.Lock;

/**
 * Class for interfacing with BNO055 sensor
 */

/*
    import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

    DeviceInterfaceModule dim;
*/

public class Gyro extends I2cDevice{

    private static final int BNO055_ADDR = 0x28;

    private static final int BNO055_ID = 0xA0;
    private static final int BNO055_ID_ADDR = 0x00;

    private static final int EUL_DATA_X_LSB_ADDR = 0x1A;
    private static final int EUL_DATA_X_MSB_ADDR = 0x1B;

    private static final int UNIT_SEL = 0x3B;
    private static final int OPR_MODE = 0x3D;
    private static final int PWR_MODE = 0x3E;
    private static final int SYS_TRIGGER = 0x3F;

    private byte[] readCache;
    private Lock readLock;
    private byte[] writeCache;
    private Lock writeLock;

    // constructor
    public Gyro(I2cController controller, int port) {
        super(controller, port);
    }

    public void getCalibrationStatus(int[] buffer) {

    }

    public void getYawData(int[] buffer) {

    }
}
