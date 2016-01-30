package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;


/**
 * Ignorant gyro test yay!
 */


public class GyroAuto extends OpMode {

    private static final int BNO055_ADDR = 0x28;

    private static final int BNO055_ID = 0xA0;
    private static final int BNO055_ID_ADDR = 0x00;

    private static final int EUL_DATA_X_LSB_ADDR = 0x1A;
    private static final int EUL_DATA_X_MSB_ADDR = 0x1B;

    private static final int UNIT_SEL = 0x3B;
    private static final int OPR_MODE = 0x3D;
    private static final int PWR_MODE = 0x3E;
    private static final int SYS_TRIGGER = 0x3F;

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    I2cDevice gyro;
    ModernRoboticsI2cGyro x;

    int state = 0;

    public void init() {

        gyro = hardwareMap.i2cDevice.get("gyro");

        readCache = gyro.getI2cReadCache();
        readLock = gyro.getI2cReadCacheLock();
        writeCache = gyro.getI2cWriteCache();
        writeLock = gyro.getI2cWriteCacheLock();

        // wait for device to boot
        delay(0.650);

    }

    public void loop() {

        delay(2);

        switch(state)
        {
            case 0:
                gyro.enableI2cReadMode(BNO055_ADDR, BNO055_ID_ADDR, 1);
                telemetry.addData("Enabled I2C read mode", "");
                state++;
                break;
            case 1:
                if(gyro.isI2cPortReady()) state++;
                else telemetry.addData("Waiting for port to be ready", "");
                break;
            case 2:
                gyro.readI2cCacheFromController();
                telemetry.addData("Read cache from controller", "");
                state++;
                break;
            case 3:
                telemetry.addData("Read cache: ", gyro.getCopyOfReadBuffer()[0]);
                break;
        }
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }

    private void performAction(String actionName, int i2cAddress, int memAddress, int memLength) {
        if (actionName.equalsIgnoreCase("read")) gyro.enableI2cReadMode(i2cAddress, memAddress, memLength);
        if (actionName.equalsIgnoreCase("write")) gyro.enableI2cWriteMode(i2cAddress, memAddress, memLength);

        gyro.setI2cPortActionFlag();
        gyro.writeI2cCacheToController();
        gyro.readI2cCacheFromController();
    }

    public void delay(double seconds)
    {
        Timer timer = new Timer(seconds);
        timer.start();
        while(!timer.done());
    }

    static public class Timer {
        long mStartTime;
        double mSeconds;

        public Timer(double seconds) {
            mStartTime = 0L;        // creation time is NOT start time
            mSeconds = seconds;
        }

        public void start() {
            mStartTime = System.nanoTime();
        }

        // return elapsed time in seconds since timer was created or restarted
        public double elapsed() {
            return (double) (System.nanoTime() - mStartTime) / (double) TimeUnit.SECONDS.toNanos(1L);
        }

        public double remaining() {
            return mSeconds - elapsed();
        }

        public boolean done() {
            return (remaining() <= 0);
        }
    }
}
