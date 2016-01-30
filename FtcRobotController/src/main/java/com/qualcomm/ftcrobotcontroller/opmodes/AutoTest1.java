package com.qualcomm.ftcrobotcontroller.opmodes;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * A test example of autonomous opmode programming using AutoLib classes.
 * Created by phanau on 12/14/15.
 */


public class AutoTest1 extends OpMode {

    AutoLib.Sequence mSequence;     // the root of the sequence tree
    boolean bDone;                  // true when the programmed sequence is done
    MediaPlayer mp;

    public AutoTest1() {
    }

    public void init() {
        mp = new MediaPlayer();
        //mp.setVolume(1.0f, 1.0f);
        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a first simple Step to the root Sequence
        mSequence.add(new AutoLib.LogTimeStep(this, "step1", 10));

        // create a ConcurrentSequence with 3 concurrent Steps
        mSequence.add(new AutoLib.TimedSongStep(mp, "/storage/emulated/0/BUCKETS.wav", 2));

        // finish up with another simple Step
        mSequence.add(new AutoLib.LogTimeStep(this, "step3", 10));

        // start out not-done
        bDone = false;
    }

    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
    }

    public void stop() {
        telemetry.addData("stop() called", "");
    }
}
