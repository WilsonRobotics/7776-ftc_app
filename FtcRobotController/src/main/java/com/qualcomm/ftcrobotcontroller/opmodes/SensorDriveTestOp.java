package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * simple example of using a Step that drives along a given azimuth for a given time
 * Created by phanau on 1/3/16.
 */


class AzimuthTimedDriveStep extends AutoLib.Step {

    private AutoLib.Timer mTimer;
    private DcMotor mMotors[];                          // the (up to 4) motors we're controlling - assumed order is fr, br, fl, bl
    private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
    private boolean mStop;                              // if true, stop motors when this step completes
    private OpMode mOpMode;                             // needed so we can log output
    private float mHeading;                             // compass heading to steer for (-180 .. +180 degrees)
    private GyroSensor mGyro;                           // sensor to use for heading information
    private SensorLib.PID pid;                          // proportional–integral–derivative controller (PID controller)
    private double mPrevTime;                           // time of previous loop() call

    public AzimuthTimedDriveStep(OpMode mode, float heading, GyroSensor sensor,
                                 DcMotor motors[], float power, float seconds, boolean stop)
    {
        mOpMode = mode;
        mHeading = heading;
        mMotors = motors;
        mPower = power;
        mTimer = new AutoLib.Timer(seconds);
        mStop = stop;
        mGyro = sensor;

        final float Kp = 0.05f;    // motor power proportional term correction per degree of deviation
        final float Ki = 0;        // ... integrator term
        final float Kd = 0;        // ... derivative term
        pid = new SensorLib.PID(Kp, Ki, Kd);
    }

    public boolean loop() {
        super.loop();

        // start the Timer on our first call
        if (firstLoopCall()) {
            mTimer.start();
            mPrevTime = mTimer.elapsed();
        }

        float heading = mGyro.getHeading();     // get latest reading from direction sensor
        // convention is positive angles CW, wrapping from 359-0

        float error = SensorLib.Utils.wrapAngle(heading-mHeading);   // deviation from desired heading
        // deviations to left are negative, to right are positive

        // compute right/left motor powers based on our current heading vs. desired heading
        float dt = (float)(mTimer.elapsed() - mPrevTime);
        float correction = pid.loop(error, dt);
        float rightPower = Range.clip(mPower + correction, -1, 1);
        float leftPower = Range.clip(mPower - correction, -1, 1);

        // set the motor powers
        if (mMotors[0] != null)
            mMotors[0].setPower(rightPower);
        if (mMotors[1] != null)
            mMotors[1].setPower(rightPower);
        if (mMotors[2] != null)
            mMotors[2].setPower(leftPower);
        if (mMotors[3] != null)
            mMotors[3].setPower(leftPower);

        // log some data
        mOpMode.telemetry.addData("heading ", heading);
        mOpMode.telemetry.addData("left power ", leftPower);
        mOpMode.telemetry.addData("right power ", rightPower);
        //if (mGyro instanceof ModernRoboticsI2cGyro)
        //    mOpMode.telemetry.addData("integrated z: ", mGyro.isCalibrating() ? "calibrating" : ((ModernRoboticsI2cGyro) mGyro).getIntegratedZValue());

        // run the motors until the Timer runs out
        boolean done = mTimer.done();
        if (done && mStop)
            for (DcMotor m : mMotors)
                if (m != null)
                    m.setPower(0);

        return done;
    }
}

// simple example sequence that tests AzimuthDriveStep to drive along a square path
public class SensorDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    DcMotor mMotors[];
    private GyroSensor mGyro;  // sensor to use for heading information

    @Override
    public void init() {

        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("front_right");
        //mMotors[1] = mf.getDcMotor("br");
        mMotors[2] = mf.getDcMotor("front_left");
        //mMotors[3] = mf.getDcMotor("bl");
        mMotors[2].setDirection(DcMotor.Direction.REVERSE);
        //mMotors[3].setDirection(DcMotor.Direction.REVERSE);

        // get hardware gyro, calibrate it, and get an initial heading from it
        mGyro = (GyroSensor) hardwareMap.gyroSensor.get("gyro");
        mGyro.calibrate();
        while (mGyro.isCalibrating()) {
            try {Thread.sleep(50);}
            catch (Exception e) {}
        }

        // wait for gyro to settle to initial reading (of zero)
        // without this we get random junk for a couple of seconds at the start (???)
        while (mGyro.getHeading() != 0);

        // create an autonomous sequence with the steps to drive
        // several laps of a polygonal course ---

        float heading = 0;      // net absolute heading resulting from sequence of relative turns
        int numLaps = 3;        // number of laps in the sequence
        int numSides = 4;       // number of sides in driven polygon
        boolean CW = false;     // go CCW
        float legTime = debug ? 10.0f : 2.0f;  // time along each leg of the polygon

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of "legs" to the sequence
        for (int i = 0; i < numLaps*numSides; i++) {

            // stop after the last step
            boolean bStop = i==(numLaps*numSides-1);

            // add a Step to the root Sequence that drives forward along current heading
            mSequence.add(new AzimuthTimedDriveStep(this, heading, mGyro, mMotors, 0.5f, legTime, bStop));

            heading += (CW ? 1 : -1) * (360/numSides);     // turn relative for next leg
        }

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
        mGyro.close();        // release the sensor we've been using for azimuth data
    }
}

