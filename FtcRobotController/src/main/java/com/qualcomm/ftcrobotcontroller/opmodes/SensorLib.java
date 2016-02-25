package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * library of utility classes supporting sensor inputs
 * Created by phanau on 1/1/16.
 */

public class SensorLib {

    public static class DirectionSensor implements SensorEventListener {

        private float[] lastMagField;
        private float[] lastAccel;
        private float mAz, mRoll, mPitch;
        private int mDataCount;         // count of how many full updates we've done

        private SensorManager mSensorManager;
        private Sensor mAccelerometer;
        private Sensor mMagneticField;

        public DirectionSensor() {
            Context ctx = PRHApp.getAppContext();
            mSensorManager = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);

            mAz = mPitch = mRoll = 0;
            mDataCount = 0;
        }

        public void init() {
            int delay = SensorManager.SENSOR_DELAY_UI;

            mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            mSensorManager.registerListener(this, mAccelerometer, delay);

            mMagneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            mSensorManager.registerListener(this, mMagneticField, delay);
        }

        public void stop() {
            mSensorManager.unregisterListener(this, mAccelerometer);
            mSensorManager.unregisterListener(this, mMagneticField);
        }

        public boolean dataReady() {
            return mDataCount > 0;
        }

        public int dataCount() {
            return mDataCount;
        }

        public float azimuth() {
            return mAz;
        }

        public float roll() {
            return mRoll;
        }

        public float pitch() {
            return mPitch;
        }

        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            Sensor mySensor = sensorEvent.sensor;

            if (mySensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                // save last Gravity reading for use by "new" orientation code
                lastMagField = sensorEvent.values.clone();
            }

            if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                lastAccel = sensorEvent.values.clone();
            }

            if (lastMagField != null && lastAccel != null) {
                // compute Orientation angles the "new" way --
                float[] outR = new float[16];
                float[] outI = new float[16];
                boolean success = SensorManager.getRotationMatrix(outR, outI, lastAccel, lastMagField);
                if (success) {
                    float[] orientVals = new float[3];
                    final float rad2deg = (float) (180.0 / Math.PI);
                    SensorManager.getOrientation(outR, orientVals);

                    // fix up results so that with phone mounted vertically along width of robot,
                    // with screen facing backwards, in landscape mode (with USB port facing up), readout gives:
                    // az (0 when robot faces W, increasing as you turn to starboard, so N=90, E=+-180, S=-90),
                    // roll (positive when roll is to right/starboard),
                    // pitch (positive when nose goes up)
                    mAz = orientVals[0] * rad2deg;
                    mRoll = orientVals[1] * rad2deg;
                    mPitch = orientVals[2] * rad2deg - 90.0f;

                    mDataCount++;       // increment count of how many full updates we've done
                }
                // discard sensor inputs and wait to recompute until we receive new ones for both
                lastAccel = null;
                lastMagField = null;
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            // TBD ...
        }

    }

    public static class FilteredDirectionSensor extends DirectionSensor {
        private FilterLib.MovingWindowAngleFilter mAzimuth, mPitch, mRoll;   // filters for az,pitch,roll data

        public FilteredDirectionSensor() {
            mAzimuth = new FilterLib.MovingWindowAngleFilter();
            mPitch = new FilterLib.MovingWindowAngleFilter();
            mRoll = new FilterLib.MovingWindowAngleFilter();
        }

        public FilteredDirectionSensor(int wsize) {
            mAzimuth = new FilterLib.MovingWindowAngleFilter(wsize);
            mPitch = new FilterLib.MovingWindowAngleFilter(wsize);
            mRoll = new FilterLib.MovingWindowAngleFilter(wsize);
        }

        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            // let the base sensor update its current data
            super.onSensorChanged(sensorEvent);

            // add the latest raw sensor values to our filters
            mAzimuth.appendValue(super.azimuth());
            mPitch.appendValue(super.pitch());
            mRoll.appendValue(super.roll());
        }

        // return filtered values of the sensor data
        public float azimuth() {
            return (float) mAzimuth.currentValue();
        }

        public float roll() {
            return (float) mRoll.currentValue();
        }

        public float pitch() {
            return (float) mPitch.currentValue();
        }

    }

    public static class Utils {

        // wrap given angle into range (-180 .. +180)
        public static float wrapAngle(float angle) {
            while (angle > 180)
                angle -= 360;
            while (angle < -180)
                angle += 360;
            return angle;
        }

    }

    public static class PID {

        private float mPrevError = 0;
        private float mIntegral = 0;
        private float mKp = 0;
        private float mKi = 0;
        private float mKd = 0;
        private float mKiCutoff = 0;     // max error value for which we integrate error over time

        public PID(float Kp, float Ki, float Kd, float KiCutoff) {
            setK(Kp, Ki, Kd, KiCutoff);
        }

        // set the parameters of the filter
        public void setK(float Kp, float Ki, float Kd, float KiCutoff)
        {
            // set filter coefficients
            mKp = Kp;
            mKi = Ki;
            mKd = Kd;

            // set threshold for errors we integrate -- integrating large errors causes instability
            mKiCutoff = KiCutoff;
        }



        // run one cycle of the PID filter given current error and delta-time since the previous call
        public float loop(float error, float dt) {
            if (Math.abs(error) < mKiCutoff)      // only integrate small errors (< 3 degrees for now)
                mIntegral += error*dt;
            float derivative = (dt > 0) ? (error - mPrevError)/dt : 0;
            float output = mKp*error + mKi*mIntegral + mKd*derivative;
            mPrevError = error;
            return output;
        }

    }

    // class that tries to correct systemic errors in ModernRoboticsI2cGyro output
    public static class CorrectedMRGyro {

        ModernRoboticsI2cGyro mGyro1, mGyro2;
        AutoLib.Timer mTimer;

        public CorrectedMRGyro(ModernRoboticsI2cGyro gyro)
        {
            // remember the (one) physical gyro we're using
            mGyro1 = gyro;
            mGyro2 = null;
        }

        public CorrectedMRGyro(ModernRoboticsI2cGyro gyro1, ModernRoboticsI2cGyro gyro2)
        {
            // remember the physical gyros we're using (where gyro2 may be null)
            mGyro1 = gyro1;
            mGyro2 = gyro2;
        }

        public void calibrate()
        {
            // calibrate the first (required) gyro
            calibrate(mGyro1);

            // if we're using two gyros, calibrate the second one
            if (mGyro2 != null) {
                calibrate(mGyro2);
            }

            // get a Timer and start it
            mTimer = new AutoLib.Timer(0);
            mTimer.start();
        }

        public void calibrate(ModernRoboticsI2cGyro gyro)
        {
            // start a calibration sequence on the gyro and wait for it to finish
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                try {Thread.sleep(100);}
                catch (Exception e) {}
            }

            // reset the on-board z-axis integrator and wait for it to zero
            gyro.resetZAxisIntegrator();
            while (gyro.getIntegratedZValue() != 0);

            // wait for gyro to settle to initial heading of zero
            // without this we get random junk for a couple of seconds at the start (???)
            // while (gyro.getHeading() != 0);      // who cares? we don't use this ...
        }

        public float getIntegratedZValue1()
        {
            // return the raw (uncorrected) integrated Z value from the underlying physical gyro1
            return mGyro1.getIntegratedZValue();
        }

        public float getIntegratedZValue2()
        {
            // return the raw (uncorrected) integrated Z value from the underlying physical gyro2 (if present)
            // if gyro2 is not present, return negative of result for gyro1, assuming g2 would be mounted
            // in the opposite z-orientation from g1 if it were present.
            return (mGyro2 != null) ? mGyro2.getIntegratedZValue() : -getIntegratedZValue1();
        }

        public float getHeading()
        {
            // since the physical gyro appears to have a small (~5%) error in the angles it reports
            // (actually about 15 degrees per revolution CCW and about 20 per rev CW),
            // we were computing the incremental angle change at each sample, correcting it for the
            // direction it's going, and then accumulating it to get the total angle change we report,
            // but now realize we can't do that -- vibration-induced jitter accumulates a bias in one direction!
            // so things worked fine in a static test, but not when motors are running :(
            // so now we just scale the cumulative integrated Z reported by the gyro using one
            // constant (18/360) for both directions and use that integrated Z to compute "heading".
            // this version supports an optional second gyro (gyro2), assumed to be mounted in the
            // opposite z-orientation to gyro1, in the hope that errors will "average out".

            float intZ1 = getIntegratedZValue1();     // positive CCW
            float intZ2 = getIntegratedZValue2();     // positive CCW
            float avgIntZ = (intZ1-intZ2)/2.0f;       // if gyro2 is absent, result is gyro1 integratedZ

            final float k = (1.0f - 18.0f/360.0f);          // correction factor: ~18 degrees / revolution
            avgIntZ *= k;

            // integrated z drifts -15 degrees per minute (-0.25 deg / sec) so add in that much correction.
            // two-gyro solution should cancel out this effect, so only do this for one gyro case
            if (mGyro2 == null) {
                final float drift = 0.25f;                      // vibration-induced drift per second
                //avgIntZ += (mTimer.elapsed() * drift);
            }

            // convert from positive CCW to positive CW like getHeading() and wrap to [0..360) range
            float heading = -avgIntZ % 360.0f;

            return heading;         // unlike Gyro interface, we return this as float, not int
        }

        public void stop()
        {
            mGyro1.close();
            if (mGyro2 != null)
                mGyro2.close();
        }
    }

}