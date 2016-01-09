package com.qualcomm.ftcrobotcontroller.opmodes;
import android.app.Application;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.PixelFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.media.AudioManager;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.Gravity;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.widget.FrameLayout;

import java.io.IOException;
import java.nio.IntBuffer;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Robotics on 12/18/2015.
 */
public class CameraTestOp {


}
// a simple class describing an RGB pixel
class Pixel {
    int mR, mG, mB;
    public Pixel(int r, int g, int b) {
        mR = r; mG = g; mB = b;
    }
    public int red() { return mR; }
    public int green() { return mG; }
    public int blue() { return mB; }
    public String toString() { return "pixel("+red()+","+green()+","+blue()+")"; }
}
// a simple wrapper around the data returned by the camera callback
// assumes the data is in RGB_565 format (for now)
class CameraImage {
    Camera.Size mSize;
    int mBpp;       // bytes per pixel
    byte[] mData;
    Bitmap mBitmap; // converted to RGB?

    public CameraImage(final byte[] imageData, Camera c) {
        mData = imageData;      // reference to (readonly) image data
        Camera.Parameters camParms = c.getParameters();
        int picFormat = camParms.getPictureFormat();
        assert (picFormat == PixelFormat.RGB_565);
        mBpp = 2; // assuming picFormat really is RGB_565
        mSize = camParms.getPictureSize();
        //mBitmap = YUVtoRGB.convert(mData, mSize.width, mSize.height);
    }
}
