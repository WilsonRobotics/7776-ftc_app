package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by phanau on 10/15/15.
 * This OpMode just echos the 2 gamepads back to the DriverStation so you can see
 * if gamepad inputs are getting to the RobotController correctly.
 * It doesn't use any other hardware, so a "NullBot" configuration works fine.
 */

public class GamepadEcho extends OpMode {

    int count = 0;

    /**
     * Constructor
     */
    public GamepadEcho() {
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
    }

    /*
 * This method will be called repeatedly in a loop
 *
 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
 */
    @Override
    public void loop() {

		/*
		 * echo values of Gamepads 1 and 2 back to driver station
		 */
        telemetry.addData("version:", "GamepadEcho 1.1 Mac");
        telemetry.addData("loop count:", count++);
        telemetry.addData("gamepad1:", gamepad1);
        telemetry.addData("gamepad2:", gamepad2);
    }

}
