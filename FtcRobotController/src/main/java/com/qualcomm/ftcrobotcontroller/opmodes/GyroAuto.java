package com.qualcomm.ftcrobotcontroller.opmodes;


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

    int port = 5;

    byte[] readCache;
    Lock readLock;
    byte[] writeCache;
    Lock writeLock;

    DeviceInterfaceModule dim;

    Timer timer;

    boolean itWorked = false;
    boolean reachedEndOfInit = false;

    public void init() {

        dim = hardwareMap.deviceInterfaceModule.get("dim");

        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);
        writeCache = dim.getI2cWriteCache(port);
        writeLock = dim.getI2cWriteCacheLock(port);

        // wait for device to boot
        timer = new Timer(0.650);
        timer.start();
        while(!timer.done());

        // make sure the port is ready
        while(!dim.isI2cPortReady(port)) {
            timer = new Timer(1);
            timer.start();
            while(!timer.done());
        }

        // read the the chip id from the BNO055
        performAction("read", port, BNO055_ADDR, BNO055_ID_ADDR, 1);

        readLock.lock();
        if(TypeConversion.unsignedByteToInt(readCache[0]) == BNO055_ADDR)
        {
            itWorked = true;
        }
        readLock.unlock();

        reachedEndOfInit = true;
    }

    public void loop() {

        if(itWorked){
            telemetry.addData("YAY", "");
        }

        if(reachedEndOfInit){
            telemetry.addData("Reached end of init()", "");
        }
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }

    private void performAction(String actionName, int port, int i2cAddress, int memAddress, int memLength) {
        if (actionName.equalsIgnoreCase("read")) dim.enableI2cReadMode(port, i2cAddress, memAddress, memLength);
        if (actionName.equalsIgnoreCase("write")) dim.enableI2cWriteMode(port, i2cAddress, memAddress, memLength);

        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);
        dim.readI2cCacheFromController(port);
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
