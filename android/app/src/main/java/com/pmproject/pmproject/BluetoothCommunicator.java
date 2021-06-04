package com.pmproject.pmproject;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;

public class BluetoothCommunicator implements Runnable {

    public static final String kDeviceName = "HC-06";

    public static final String kMsgMB_MobileOn = "mobile_on";
    public static final String kMsgMB_CalibrationDone = "calibration_done";
    public static final String kMsgMB_PositionFormat = "p %d %d";
    public static final String kMsgMB_DirectionFormat = "d %d";
    public static final int kMsgPositionMultiplier = 10000;

    public static final String kMsgBM_Calibrate = "calibrate";
    public static final String kMsgBM_LogOff = "log_off";
    public static final String kMsgBM_LogPosition = "log_position";
    public static final String kMsgBM_LogDirection = "log_direction";
    public static final int kDelayBetweenPositionsMillis = 1000;

    private final Thread thread;

    private BluetoothSocket socket = null;
    private InputStream input = null;
    private OutputStream output = null;
    private final Queue<String> messages = new LinkedBlockingQueue<>();
    private boolean running;

    public BluetoothCommunicator()
    {
        connectToDevice();
        // sendMessage(kMsgMB_MobileOn);
        running = true;
        thread = new Thread(this);
        thread.start();
    }

    private void connectToDevice()
    {
        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        Set<BluetoothDevice> devices = adapter.getBondedDevices();
        BluetoothDevice device = null;

        for (BluetoothDevice d : devices)
            if (d.getName().equals(kDeviceName))
                device = d;

        if (device == null) {
            Log.e(MainActivity.TAG, "Could not find a BT device with the matching name " + kDeviceName);
            return;
        }

        try {
            socket = device.createInsecureRfcommSocketToServiceRecord(
                    device.getUuids()[0].getUuid());
            socket.connect();
            input = socket.getInputStream();
            output = socket.getOutputStream();
        } catch (IOException e) {
            e.printStackTrace();
            if (socket != null) {
                try {
                    socket.close();
                } catch (IOException ioException) {
                    ioException.printStackTrace();
                }
            }
        }
    }

    public void stop()
    {
        running = false;
        try {
            socket.close();
            thread.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        List<Byte> bytes = new ArrayList<>();
        while (running)
        {
            try {
                int b = input.read();
                Log.i(MainActivity.TAG, "Received '" + (char) b + "' code " + b);
                if (b == -1)
                {
                    Log.e(MainActivity.TAG, "Received -1 from BT!!");
                    continue;
                }
                if (b == '\n')
                {
                    byte[] bytesArray = new byte[bytes.size()];
                    for (int i = 0; i < bytes.size(); i++)
                        bytesArray[i] = bytes.get(i);
                    String message = new String(bytesArray, StandardCharsets.UTF_8);
                    // Log.i(MainActivity.TAG, "Received '" + message + "'");
                    messages.add(message);
                    bytes.clear();
                }
                else
                {
                    bytes.add((byte)b);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public String pollMessage()
    {
        return messages.poll();
    }

    public void sendMessage(String msg)
    {
        Log.i(MainActivity.TAG, "Sending '" + msg + "' to board");
        sendMessageRaw(msg + "\n");
    }

    private void sendMessageRaw(String msg)
    {
        try {
            output.write(msg.getBytes());
            output.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
