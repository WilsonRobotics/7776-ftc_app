package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Wrapper around Motor to encapsulate Encoder usage
 * Created by phanau on 11/28/15.
 */

public class EncoderMotor {

    private DcMotor m_motor;

    // constructor - takes the DcMotor we'll be controlling
    public EncoderMotor(DcMotor motor) {
        m_motor = motor;
    }

    // set power setting of the motor - returns true if motor controller was ready to be written to
    public boolean setPower(double power) {
        boolean okay = setToWrite();
        if (okay)
            m_motor.setPower(power);
        return okay;
    }

    // get power setting of the motor - returns 0 if motor controller was not (yet) readable
    public double getPower() {
        if (setToRead())
            return m_motor.getPower();
        else
            return 0;
    }

    // set the motor to run using encoder
    public boolean runUsingEncoder() {
        boolean okay = setToWrite();
        if (okay)
            m_motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        return okay;
    }

    // reset the motor's encoder
    public boolean resetEncoder() {
        boolean okay = setToWrite();
        if (okay)
            m_motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        return okay;
    }

    // returns true when the encoder has reset
    public boolean hasEncoderReset() {
        // Has the encoder reached zero?
        return (encoderCount() == 0);
    }

    // set the motor to run without using encoder
    public boolean runWithoutEncoder() {
        boolean okay = setToWrite();
        if (okay)
            m_motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        return okay;
    }

    // get the motor's current encoder count
    public int encoderCount() {
        if (isReadable())
            return m_motor.getCurrentPosition();
        else
            return 0;       // use this value to indicate "not ready", assuming we'll never ask for it as a target
    }

    // returns true if the motor has reached its encoder count
    public boolean hasEncoderReached(int count) {
        return (Math.abs(encoderCount()) >= count);
    }

    // set controller mode to READ -- USB controllers always return true, old NXT may need to wait
    boolean setToRead() {
        DcMotorController.DeviceMode dm = getDeviceMode();

        // if it's already in a readable mode (which new controllers always are), we're done
        if (isReadable())
            return true;

        // it must be an old style NXT controller ...
        if (dm != DcMotorController.DeviceMode.SWITCHING_TO_READ_MODE)          // not already "in transition" ...
            setDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        return false;       // not ready yet
    }

    // set controller mode to WRITE -- USB controllers always return true, old NXT may need to wait
    boolean setToWrite() {
        DcMotorController.DeviceMode dm = getDeviceMode();

        // if it's already in a writable mode (which new controllers always are), we're done
        if (isWritable())
            return true;

        // it must be an old style NXT controller ...
        if (dm != DcMotorController.DeviceMode.SWITCHING_TO_WRITE_MODE)          // not already "in transition" ...
            setDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        return false;       // not ready yet
    }

    // check to see if the controller is readable
    public boolean isReadable() {
        DcMotorController.DeviceMode dm = getDeviceMode();
        return dm == DcMotorController.DeviceMode.READ_ONLY || dm == DcMotorController.DeviceMode.READ_WRITE;
    }

    // check to see if the controller is writable
    public boolean isWritable() {
        DcMotorController.DeviceMode dm = getDeviceMode();
        return dm == DcMotorController.DeviceMode.WRITE_ONLY || dm == DcMotorController.DeviceMode.READ_WRITE;
    }

    // get controller device mode -- USB controllers always return READ_WRITE, old NXT is more complex
    DcMotorController.DeviceMode getDeviceMode() {
        DcMotorController mc = m_motor.getController();
        return mc.getMotorControllerDeviceMode();
    }

    // set controller device mode -- USB controllers are always READ_WRITE, old NXT is more complex
    void setDeviceMode(DcMotorController.DeviceMode dm) {
        DcMotorController mc = m_motor.getController();
        mc.setMotorControllerDeviceMode(dm);
    }
}


/* not used

    // tell the motor to run at the given power to the given encoder count
    public void driveToCountInit(double power) {
        // Tell the system that motor encoder will be used.
        runUsingEncoder();

        // Start the motor at given power.
        setPower(power);
    }

    // call this function from loop() until it returns true
    public boolean driveToCountLoop(double count)
    {
        // Has the encoder reached the limit? Assume not.
        boolean l_return = false;

        // Has the motor shaft turned the required amount?
        if (hasEncoderReached (count))
        {
            // Reset the encoder to ensure it is at a known good value.
            resetEncoder ();

            // Stop the motor.
            setPower (0.0);

            // Indicate this operation is completed.
            l_return = true;
        }

        // Return the status.
        return l_return;

    }
*/
