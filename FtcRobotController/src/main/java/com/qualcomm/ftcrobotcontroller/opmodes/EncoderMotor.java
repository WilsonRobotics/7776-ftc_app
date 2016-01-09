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

    // accessor if client needs to retrieve underlying motor
    // public DcMotor motor() { return m_motor; }

    // set/get power setting of the motor
    public void setPower(double power) {
        m_motor.setPower(power);
    }

    public double getPower() {
        return m_motor.getPower();
    }

    // set the motor to run using encoder
    public void runUsingEncoder() {
        m_motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    // reset the motor's encoder
    public void resetEncoder() {
        m_motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    // returns true when the encoder has reset
    public boolean hasEncoderReset() {
        // Has the encoder reached zero?
        return (encoderCount() == 0);
    }

    // set the motor to run without using encoder
    public void runWithoutEncoder() {
        if (m_motor.getMode() == DcMotorController.RunMode.RESET_ENCODERS) {
            m_motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
    }

    // get the motor's current encoder count
    public int encoderCount() {
        return m_motor.getCurrentPosition();
    }

    // returns true if the motor has reached its encoder count
    public boolean hasEncoderReached(double p_count) {
        return (Math.abs(encoderCount()) >= p_count);
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
