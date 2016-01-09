package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * simple example of using a Step that drives along a given azimuth for a given time
 * Created by phanau on 1/3/16.
 */


class AzimuthTimedDriveStep extends AutoLib.Step {
    private AutoLib.Timer mTimer;
    private DcMotor mMotors[];                          // the (4) motors we're controlling - assumed order is fr, br, fl, bl
    private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
    private boolean mStop;                              // if true, stop motors when this step completes
    private OpMode mOpMode;                             // needed so we can log output
    private float mHeading;                             // compass heading to steer for (-180 .. +180 degrees)
    private SensorLib.FilteredDirectionSensor mFDS;     // sensor to use for azimuth input

    public AzimuthTimedDriveStep(float heading, SensorLib.FilteredDirectionSensor sensor,
                                 DcMotor motors[], float power, float seconds, boolean stop)
    {
        mHeading = heading;
        mMotors = motors;
        mPower = power;
        mTimer = new AutoLib.Timer(seconds);
        mStop = stop;
        mFDS = sensor;
    }

    public boolean loop() {
        super.loop();

        // start the Timer on our first call
        if (firstLoopCall()) {
            mTimer.start();
        }

        // compute motor power based on our current heading vs. desired heading
        float heading = mFDS.azimuth();     // get latest reading from filtered direction sensor
        float diff = SensorLib.Utils.wrapAngle(heading-mHeading);   // deviation from desired heading
        // deviations to left are negative, to right are positive
        final float corrPower = 0.005f;       // motor power correction per degree of deviation
        float rightPower = mPower + diff*corrPower;
        float leftPower = mPower - diff*corrPower;

        // set the appropriate motor powers
        mMotors[0].setPower(rightPower);
        mMotors[1].setPower(rightPower);
        mMotors[2].setPower(leftPower);
        mMotors[3].setPower(leftPower);

        // check to see if we're in danger of tipping over
        boolean danger = (Math.abs(mFDS.pitch()) > 30) || (Math.abs(mFDS.roll()) > 30);

        // run the motors until the Timer runs out or we sense danger
        boolean done = mTimer.done();
        if ((done && mStop) || danger)
            for (DcMotor m : mMotors)
                m.setPower(0);

        return done | danger;       // force "done" when there's danger
    }
}

// simple example sequence that tests AzimuthDriveStep to drive along a square path
public class SensorDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    DcMotor mMotors[];
    SensorLib.FilteredDirectionSensor mFDS;

    @Override
    public void init() {
        AutoLib.HardwareFactory mf = null;
        final boolean debug = true;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("fr");
        mMotors[1] = mf.getDcMotor("br");
        mMotors[2] = mf.getDcMotor("fl");
        mMotors[3] = mf.getDcMotor("bl");
        mMotors[2].setDirection(DcMotor.Direction.REVERSE);
        mMotors[3].setDirection(DcMotor.Direction.REVERSE);

        // create a filtered direction sensor and get an initial heading from it
        final int filterSize = 10;
        mFDS = new SensorLib.FilteredDirectionSensor(filterSize);
        mFDS.init();
        while(mFDS.dataCount() < filterSize)
            ;  // just wait until data settles ...
        float initialHeading = mFDS.azimuth();

        // create an autonomous sequence with the necessary steps to drive a square course

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        float legTime = debug ? 10.0f : 2.5f;        // time along each leg of the square

        // add a Step to the root Sequence that drives forward along our initial heading
        mSequence.add(new AzimuthTimedDriveStep(initialHeading, mFDS, mMotors, 0.5f, legTime, false));

        // add a second Step that turns left 90 degrees and drives along that course
        mSequence.add(new AzimuthTimedDriveStep(SensorLib.Utils.wrapAngle(initialHeading - 90), mFDS, mMotors, 0.5f, legTime, false));

        // add a third Step that turns left 90 degrees more and drives along that course
        mSequence.add(new AzimuthTimedDriveStep(SensorLib.Utils.wrapAngle(initialHeading - 180), mFDS, mMotors, 0.5f, legTime, false));

        // add a fourth Step that turns left 90 degrees more and drives along that course
        mSequence.add(new AzimuthTimedDriveStep(SensorLib.Utils.wrapAngle(initialHeading - 270), mFDS, mMotors, 0.5f, legTime, true));

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
        mFDS.stop();        // release the sensor we've been using for azimuth data
    }
}
