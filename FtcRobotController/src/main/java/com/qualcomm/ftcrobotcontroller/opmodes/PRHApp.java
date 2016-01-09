package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Application;
import android.content.Context;

/**
 * Created by phanau on 12/13/15.
 */

// give OpMode access to an application Context
    // http://stackoverflow.com/questions/2002288/static-way-to-get-context-on-android

public class PRHApp extends Application {

    private static Context context;

    public void onCreate() {
        super.onCreate();
        PRHApp.context = getApplicationContext();
    }

    public static Context getAppContext() {
        return PRHApp.context;
    }

}
