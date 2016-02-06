package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

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
                    // with screen facing backwards, in landscape mode (with USB port facing down), readout gives:
                    // az (0 when robot faces E, increasing as you turn to starboard, so S=90, W=+-180, N=-90),
                    // roll (positive when roll is to right/starboard),
                    // pitch (positive when nose goes up)
                    mAz = orientVals[0] * rad2deg;
                    mRoll = -orientVals[1] * rad2deg;
                    mPitch = -orientVals[2] * rad2deg - 90.0f;

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

        public PID(float Kp, float Ki, float Kd) {
            mKp = Kp;
            mKi = Ki;
            mKd = Kd;
        }

        public float loop(float error, float dt) {
            mIntegral += error*dt;
            float derivative = (dt > 0) ? (error - mPrevError)/dt : 0;
            float output = mKp*error + mKi*mIntegral + mKd*derivative;
            mPrevError = error;
            return output;
        }

    }

}