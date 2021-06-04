package com.pmproject.pmproject;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.*;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.util.Locale;

public class MainActivity extends AppCompatActivity implements View.OnTouchListener, CameraBridgeViewBase.CvCameraViewListener2 {

    public enum LogType
    {
        LOG_OFF,
        LOG_POSITION,
        LOG_DIRECTION
    }

    public static final String TAG = "MainActivity";
    private CameraBridgeViewBase mOpenCvCameraView;
    private ImageProcessor imageProcessor;
    private BluetoothCommunicator bluetooth;
    private LogType logType = LogType.LOG_OFF;
    private long lastPositionSentAt = 0;

    private final BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        public void onManagerConnected(int status) {
            if (status == SUCCESS) {
                Log.i(TAG, "OpenCV loaded successfully");
                mOpenCvCameraView.enableView();
                mOpenCvCameraView.setOnTouchListener(MainActivity.this);
            }
            else
            {
                super.onManagerConnected(status);
            }
        }
    };

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //Check if permission is already granted
        //thisActivity is your activity. (e.g.: MainActivity.this)
        if (ContextCompat.checkSelfPermission(this,  Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {

            // Give first an explanation, if needed.
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.CAMERA)) {
                // Show an explanation to the user *asynchronously* -- don't block
                // this thread waiting for the user's response! After the user
                // sees the explanation, try again to request the permission.
            } else {
                // No explanation needed, we can request the permission.
                ActivityCompat.requestPermissions(this, new String[] { Manifest.permission.CAMERA },1);
            }
        }

        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        mOpenCvCameraView = findViewById(R.id.surf_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setMaxFrameSize(ImageProcessor.kOrigWidth, ImageProcessor.kOrigHeight);

        imageProcessor = new ImageProcessor(this::onDirectionChanged, this::onCalibrationDone);
        bluetooth = new BluetoothCommunicator();
    }

    private Void onCalibrationDone(Void ignored)
    {
        bluetooth.sendMessage(BluetoothCommunicator.kMsgMB_CalibrationDone);
        return null;
    }
    
    private Void onDirectionChanged(int direction)
    {
        if (logType == LogType.LOG_DIRECTION) {
            String msg = String.format(Locale.ENGLISH,  BluetoothCommunicator.kMsgMB_DirectionFormat, direction);
            bluetooth.sendMessage(msg);
        }
        return null;
    }

    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal  OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        /// TODO("Not yet implemented")
    }

    public void onCameraViewStopped() {
        // TODO("Not yet implemented")
    }

    private void handleMessages()
    {
        while (true)
        {
            String message = bluetooth.pollMessage();
            if (message == null)
                break;
            message = message.trim();
            Log.i(TAG, "Received '" + message + "'");
            if (message.equals(BluetoothCommunicator.kMsgBM_Calibrate)) {
                logType = LogType.LOG_OFF;
                imageProcessor.startCalibrating();
            } else if (message.equals(BluetoothCommunicator.kMsgBM_LogOff)) {
                logType = LogType.LOG_OFF;
            } else if (message.equals(BluetoothCommunicator.kMsgBM_LogDirection)) {
                imageProcessor.resetTracker();
                logType = LogType.LOG_DIRECTION;
            } else if (message.equals(BluetoothCommunicator.kMsgBM_LogPosition)) {
                logType = LogType.LOG_POSITION;
            } else {
                Log.w(TAG, "Unknown message received: '" + message + "'");
            }
        }
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        handleMessages();
        Mat output = imageProcessor.processFrame(inputFrame);
        if (logType == LogType.LOG_POSITION) {

            long now = System.currentTimeMillis();

            if (now - lastPositionSentAt >=
                    BluetoothCommunicator.kDelayBetweenPositionsMillis) {

                Vec2 pointer = imageProcessor.getPointer();
                int x = -1, y = -1;
                if (pointer != null) {
                    x = (int) Math.round((pointer.x / ImageProcessor.kSmallWidth)
                            * BluetoothCommunicator.kMsgPositionMultiplier);
                    y = (int) Math.round((pointer.y / ImageProcessor.kSmallHeight)
                            * BluetoothCommunicator.kMsgPositionMultiplier);
                }
                String msg = String.format(Locale.ENGLISH, BluetoothCommunicator.kMsgMB_PositionFormat, x, y);
                bluetooth.sendMessage(msg);

                if (now - lastPositionSentAt >=
                        2 * BluetoothCommunicator.kDelayBetweenPositionsMillis) {
                    lastPositionSentAt = now;
                } else {
                    lastPositionSentAt += BluetoothCommunicator.kDelayBetweenPositionsMillis;
                }
            }
        }
        return output;
    }

    @SuppressLint("ClickableViewAccessibility")
    public boolean onTouch(View v, MotionEvent event) {
        return false;
    }
}