package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Created by phanau on 12/14/15.
 */

// a library of classes that support autonomous opmode programming
public class AutoLib {

    // the base class from which everything else derives.
    // each action in an autonomous sequence is a Step of some kind.
    // a Step may be simple (like run a Motor) or a composite of several Steps which
    // are either run sequentially or in parallel (see Sequences below).
    static public abstract class Step {

        int mLoopCount;     // keeps count of how many times loop() has been called on this Step

        protected Step() {
            mLoopCount = 0;
        }

        // returns true iff called from the first call to loop() on this Step
        boolean firstLoopCall() {
            boolean flc = (mLoopCount == 1);    // assume this is called AFTER super.loop()
            mLoopCount++;
            return flc;
        }

        // run the next time-slice of the Step; return true when the Step is completed
        boolean loop() {
            mLoopCount++;       // increment the loop counter
            return false;
        }

    }

    // ------------------ some implementations of Sequence constructs -------------------------

    // base class for Sequences that perform multiple Steps, either sequentially or concurrently
    static public abstract class Sequence extends Step {
        protected ArrayList<Step> mSteps;  // expandable array containing the Steps in the Sequence

        protected Sequence() {
            mSteps = new ArrayList<Step>(10);   // create the array with an initial capacity of 10
        }

        // add a Step to the Sequence
        public Step add(Step step) {
            mSteps.add(step);
            return this;        // allows daisy-chaining of calls
        }

        // run the next time-slice of the Sequence; return true when the Sequence is completed.
        public boolean loop() {
            super.loop();
            return false;
        }

    }

    // a Sequence that performs its constituent Steps sequentially
    static public class LinearSequence extends Sequence {
        int mIndex;     // index of currently active Step

        public LinearSequence() {
            mIndex = 0;     // start at the beginning
        }

        // run the current Step of the Sequence until it completes, then the next Step and
        // the next, etc., etc. until the last Step completes, at which point the Sequence
        // returns complete.
        public boolean loop() {
            super.loop();
            if (mIndex < mSteps.size()) {       // if this Sequence is not completed ...
                if (mSteps.get(mIndex).loop())  // if this Step is complete, move to the next Step
                    mIndex++;
            }
            return (mIndex >= mSteps.size());   // return true when last Step completes
        }

    }


    // a Sequence that performs its constituent Steps concurrently
    static public class ConcurrentSequence extends Sequence {

        public ConcurrentSequence() {
        }

        // run all the Steps in the Sequence "concurrently" -- i.e. run the loop() function of
        // each of the Steps each time loop() is called. When ALL the Steps report that they
        // are done, then this Sequence is done.
        public boolean loop() {
            super.loop();
            boolean bDone = true;
            for (Step s : mSteps)
                bDone &= s.loop();      // "done" means ALL Steps are done
            return bDone;
        }

    }


    // ------------------ some implementations of primitive Steps ----------------------------

    // a simple Step that just logs its existence for a given number of loop() calls
    // really just for testing sequencer stuff, not for actual use on a robot.
    static public class LogCountStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        int mCount;         // current loop count of this Step

        public LogCountStep(OpMode opMode, String name, int loopCount) {
            mOpMode = opMode;
            mName = name;
            mCount = loopCount;
        }

        public boolean loop() {
            super.loop();

            // log some info about this Step
            if (mCount > 0) {
                mOpMode.telemetry.addData(mName, "count = " + mCount);
                mCount--;

                // wait a bit so we can see the displayed info ...
                try {
                    Thread.sleep(500);
                } catch (Exception e) {
                }
            } else
                mOpMode.telemetry.addData(mName, "done");


            // return true when count is exhausted
            return (mCount <= 0);
        }

    }


    // a simple Step that just logs its existence for a given length of time
    static public class LogTimeStep extends Step {
        OpMode mOpMode;     // needed so we can log output
        String mName;       // name of the output field
        Timer mTimer;       // Timer for this Step

        public LogTimeStep(OpMode opMode, String name, double seconds) {
            mOpMode = opMode;
            mName = name;
            mTimer = new Timer(seconds);
        }

        public boolean loop() {
            super.loop();

            // start the Timer on our first call
            if (firstLoopCall())
                mTimer.start();

            // log some info about this Step every nth call (to not slow things down too much)
            if (!mTimer.done() && mLoopCount % 100 == 0)        // appears to cycle here at about 3ms/loop
                mOpMode.telemetry.addData(mName, "time = " + mTimer.remaining());
            if (mTimer.done())
                mOpMode.telemetry.addData(mName, "done");

            // return true when time is exhausted
            return (mTimer.done());
        }

    }


    // a Step that runs a DcMotor at a given power, for a given time
    static public class TimedMotorStep extends Step {
        Timer mTimer;
        DcMotor mMotor;
        double mPower;
        boolean mStop;          // stop motor when count is reached

        public TimedMotorStep(DcMotor motor, double power, double seconds, boolean stop) {
            mMotor = motor;
            mPower = power;
            mTimer = new Timer(seconds);
            mStop = stop;
        }

        public boolean loop() {
            super.loop();

            // start the Timer and start the motor on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mMotor.setPower(mPower);
            }

            // run the motor at the requested power until the Timer runs out
            boolean done = mTimer.done();
            if (done && mStop)
                mMotor.setPower(0);

            return done;
        }

    }


    // a Step that runs a DcMotor at a given power, for a given encoder count
    static public class EncoderMotorStep extends Step {
        EncoderMotor mMotor;    // motor to control
        double mPower;          // power level to use
        double mEncoderCount;   // target encoder count
        int mState;             // internal state machine state
        boolean mStop;          // stop motor when count is reached

        public EncoderMotorStep(EncoderMotor motor, double power, double count, boolean stop) {
            mMotor = motor;
            mPower = power;
            mEncoderCount = count;
            mState = 0;
            mStop = stop;
        }

        public boolean loop() {
            super.loop();

            boolean done = false;

            // we need a little state machine to make the encoders happy
            switch (mState) {
                case 0:
                    // reset the encoder on our first call
                    mMotor.resetEncoder();
                    mState++;
                    break;
                case 1:
                    // stay in this state until encoder has finished resetting
                    if (mMotor.hasEncoderReset())
                        mState++;
                    break;
                case 2:
                    // enable encoder and set motor power on second call
                    mMotor.runUsingEncoder();
                    mMotor.setPower(mPower);
                    mState++;
                    break;
                default:
                    // the rest of the time, just check to see if we're done
                    done = mMotor.hasEncoderReached(mEncoderCount);
                    if (done && mStop)
                        mMotor.setPower(0);     // optionally stop motor when target reached
                    break;
            }

            return done;
        }

    }

    //a class to run a servo to a place for a set time
    static public class TimedServoStep extends Step {
        Timer mTimer;
        Servo mServo;
        double mPosition;
        boolean mReturn;          // stop motor when count is reached
        double lastPosition;

        public TimedServoStep(Servo servo, double position, double seconds, boolean goBack) {
            mServo = servo;
            mPosition = position;
            mTimer = new Timer(seconds);
            mReturn = goBack;
            lastPosition = mServo.getPosition();
        }

        public boolean loop() {
            super.loop();

            // start the Timer and start the motor on our first call
            if (firstLoopCall()) {
                mTimer.start();
                mServo.setPosition(mPosition);
            }

            // run the motor at the requested power until the Timer runs out
            boolean done = mTimer.done();
            if (done && mReturn)
               mServo.setPosition(lastPosition);

            return done;
        }

    }


    // some convenience utility classes for common operations

    // a Sequence that moves a four-motor robot in a straight line with given power for given time
    static public class MoveByTime extends ConcurrentSequence {

        public MoveByTime(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, double seconds, boolean stop) {
            if(fr != null)
                this.add(new TimedMotorStep(fr, power, seconds, stop));
            if(br != null)
                this.add(new TimedMotorStep(br, power, seconds, stop));
            if(fl != null)
                this.add(new TimedMotorStep(fl, power, seconds, stop));
            if(bl != null)
                this.add(new TimedMotorStep(bl, power, seconds, stop));
        }

    }


    // a Sequence that turns a four-motor robot by applying the given right and left powers for given time
    static public class TurnByTime extends ConcurrentSequence {

        public TurnByTime(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, double seconds, boolean stop) {
            if(fr != null)
                this.add(new TimedMotorStep(fr, rightPower, seconds, stop));
            if(br != null)
                this.add(new TimedMotorStep(br, rightPower, seconds, stop));
            if(fl != null)
                this.add(new TimedMotorStep(fl, leftPower, seconds, stop));
            if(fr != null)
                this.add(new TimedMotorStep(bl, leftPower, seconds, stop));
        }

    }


    // a Sequence that moves a four-motor robot in a straight line with given power for given encoder count
    static public class MoveByEncoder extends ConcurrentSequence {

        public MoveByEncoder(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double power, double count, boolean stop) {
            if(fr != null)
                this.add(new EncoderMotorStep(new EncoderMotor(fr), power, count, stop));
            if(br != null)
                this.add(new EncoderMotorStep(new EncoderMotor(br), power, count, stop));
            if(fl != null)
                this.add(new EncoderMotorStep(new EncoderMotor(fl), power, count, stop));
            if(bl != null)
                this.add(new EncoderMotorStep(new EncoderMotor(bl), power, count, stop));
        }

    }


    // a Sequence that turns a four-motor robot by applying the given right and left powers for given right and left encoder counts
    static public class TurnByEncoder extends ConcurrentSequence {

        public TurnByEncoder(DcMotor fr, DcMotor br, DcMotor fl, DcMotor bl, double rightPower, double leftPower, double rightCount, double leftCount, boolean stop) {
            if(fr != null)
                this.add(new EncoderMotorStep(new EncoderMotor(fr), rightPower, rightCount, stop));
            if(br != null)
                this.add(new EncoderMotorStep(new EncoderMotor(br), rightPower, rightCount, stop));
            if(fl != null)
                this.add(new EncoderMotorStep(new EncoderMotor(fl), leftPower, leftCount, stop));
            if(bl != null)
                this.add(new EncoderMotorStep(new EncoderMotor(bl), leftPower, leftCount, stop));
        }

    }


    // timer
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

    // test hardware classes -- useful for testing when no hardware is available.
    // these are primarily intended for use in testing autonomous mode code, but could
    // also be useful for testing tele-operator modes.

    // a dummy DcMotor that just logs commands we send to it --
    // useful for testing Motor code when you don't have real hardware handy
    static public class TestMotor extends DcMotor {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this motor
        double mPower;      // current power setting

        public TestMotor(String name, OpMode opMode) {
            super(null, 0);     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPower = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPower(double power) {
            mPower = power;
            mOpMode.telemetry.addData(mName, " power: " + String.valueOf(mPower));
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPower() {
            return mPower;
        }
    }

    // a dummy Servo that just logs commands we send to it --
    // useful for testing Servo code when you don't have real hardware handy
    static public class TestServo extends Servo {
        OpMode mOpMode;     // needed for logging data
        String mName;       // string id of this servo
        double mPosition;   // current target position

        public TestServo(String name, OpMode opMode) {
            super(null, 0);     // init base class (real DcMotor) with dummy data
            mOpMode = opMode;
            mName = name;
            mPosition = 0.0;
        }

        @Override       // this function overrides the setPower() function of the real DcMotor class
        public void setPosition(double position) {
            mPosition = position;
            mOpMode.telemetry.addData(mName, " position: " + String.valueOf(mPosition));
        }

        @Override       // this function overrides the getPower() function of the real DcMotor class
        public double getPosition() {
            return mPosition;
        }
    }

    // define interface to Factory that creates various kinds of hardware objects
    static public interface HardwareFactory {
        public DcMotor getDcMotor(String name);
        public Servo getServo(String name);
    }

    // this implementation generates test-hardware objects that just log data
    static public class TestHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for logging data

        public TestHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            return new TestMotor(name, mOpMode);
        }

        public Servo getServo(String name){
            return new TestServo(name, mOpMode);
        }
    }

    // this implementation gets real hardware objects from the hardwareMap of the given OpMode
    static public class RealHardwareFactory implements HardwareFactory {
        OpMode mOpMode;     // needed for access to hardwareMap

        public RealHardwareFactory(OpMode opMode) {
            mOpMode = opMode;
        }

        public DcMotor getDcMotor(String name){
            return mOpMode.hardwareMap.dcMotor.get(name);
        }

        public Servo getServo(String name){
            return mOpMode.hardwareMap.servo.get(name);
        }
    }

}



