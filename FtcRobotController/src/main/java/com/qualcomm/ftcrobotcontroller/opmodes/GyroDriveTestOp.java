package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * simple example of using a Step that uses gyro input to drive along a given course for a given time
 * Created by phanau on 1/3/16 as SensorDriveTestOp, modified 1/22/16 to use Gyro, renamed GyroDriveTestOp 2/16/16.
**/


// simple example sequence that tests either of gyro-based AzimuthCountedDriveStep or AzimuthTimedDriveStep to drive along a square path
public class GyroDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    navXGyro mGyro;
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    Servo leftArm;
    Servo rightArm;
    Servo bucket;

    private static final String release1Name = "dropit1";
    private static final String release2Name = "dropit2";
    private static final String bucketName = "bucket";
    private static final double flappyArmPowerUp = 0.65;
    private static final double flappyArmPowerDown = 0.1;
    private static final double rightOffset = 0.08; //negative offset for the right servo
    private static final double bucketPowerUp = 0;
    private static final double bucketPowerDown = 0.90;

    // parameters of the PID controller for this sequence
    float Kp = 0.10f;        // motor power proportional term correction per degree of deviation
    float Ki = 0.08f;         // ... integrator term
    float Kd = 0;             // ... derivative term
    float KiCutoff = 0.0f;    // maximum angle error for which we update integrator

    @Override
    public void init() {
        bSetup = false;      // start out in Kp/Ki setup mode
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        leftArm = hardwareMap.servo.get(release1Name);
        rightArm = hardwareMap.servo.get(release2Name);
        bucket = hardwareMap.servo.get(bucketName);
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);
        bucket.setPosition(bucketPowerDown);
        leftArm.setPosition(flappyArmPowerDown);
        rightArm.setPosition(flappyArmPowerDown - rightOffset);


        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        (mMotors[0] = mf.getDcMotor("front_right")).setDirection(DcMotor.Direction.REVERSE);
        mMotors[1] = null;
        (mMotors[2] = mf.getDcMotor("front_left")).setDirection(DcMotor.Direction.FORWARD);
        mMotors[3] = null;

        // get hardware gyro(s)
        mGyro = new navXGyro(hardwareMap.deviceInterfaceModule.get("dim"), 0);
        telemetry.addData("Offset: ", mGyro.calibrate());


        // create a PID controller for the sequence
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float power = 1.0f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        boolean bUseEncoders = false;
        if (bUseEncoders) {
            // add a bunch of encoder-counted "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
            // wrap motors to simplify use of encoders
            EncoderMotor[] mEM = new EncoderMotor[4];
            for (int i=0; i<4; i++){
                if(mMotors[i] != null) mEM[i] = new EncoderMotor(mMotors[i]);
                else mEM[i] = null;
            }
            int leg = 5000;  // motor-encoder count along each leg of the polygon

            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -90, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -180, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -270, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, -90, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 90, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 180, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 270, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 0, mGyro, mPid, mEM, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 90, mGyro, mPid, mEM, power, leg, false));
            mSequence.add(new AutoLib.AzimuthCountedDriveStep(this, 180, mGyro, mPid, mEM, power, leg * 4, true));
        }
        else {
            // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
            float leg = debug ? 10.0f : 5.0f;  // time along each leg of the polygon
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -90, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -180, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -270, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, -90, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 90, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 180, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 270, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 0, mGyro, mPid, mMotors, power, leg * 2, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 90, mGyro, mPid, mMotors, power, leg, false));
            mSequence.add(new AutoLib.AzimuthTimedDriveStep(this, 180, mGyro, mPid, mMotors, power, leg * 4, true));
        }

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {

        if (gamepad1.y)
            bSetup = true;      // enter "setup mode" using controller inputs to set Kp and Ki
        if (gamepad1.x)
            bSetup = false;     // exit "setup mode"

        if (bSetup) {           // "setup mode"
            // adjust PID parameters by joystick inputs
            Kp -= (gamepad1.left_stick_y * 0.0001f);
            Ki -= (gamepad1.right_stick_y * 0.0001f);
            // update the parameters of the PID used by all Steps in this test
            mPid.setK(Kp, Ki, Kd, KiCutoff);
            // log updated values to the operator's console
            telemetry.addData("Kp = ", Kp);
            telemetry.addData("Ki = ", Ki);
            return;
        }

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
        mGyro.stop();        // release the physical sensor(s) we've been using for azimuth data
    }
}

