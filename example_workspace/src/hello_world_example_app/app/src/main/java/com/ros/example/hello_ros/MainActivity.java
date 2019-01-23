package com.ros.example.hello_ros;

import android.Manifest;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.lang.Runnable;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;

public class MainActivity extends Activity {
    static {
        System.loadLibrary("native-activity");
    }

    private final String TAG = "HELLO-WORLD-EXAMPLE";

    private enum Status {
        WAITING, RUNNING
    }

    private class RosThread implements Runnable {
        public RosThread() {
            Log.i(TAG, "calling __RosThread");
            __RosThread();
            Log.i(TAG, "ended __RosThread");
        }
        @Override
        public native void run();
        public native void stop();
        public native boolean checkRosMaster(String masterIP, String myIP);
        private native void __RosThread();
    }

    private RosThread mainThread;
    private EditText masterIP;
    private EditText myIP;
    private Button runButton;
    private TextView statusText;
    private Status status;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "OnCreate()");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.logging);

        masterIP = (EditText) findViewById(R.id.master_ip);
        myIP = (EditText) findViewById(R.id.my_ip);
        runButton = (Button) findViewById(R.id.run_button);
        statusText = (TextView) findViewById(R.id.status);
        status = Status.WAITING;

        Log.i(TAG, "new RosThread");
        mainThread = new RosThread();
        Log.i(TAG, "end RosThread constructor");

        runButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                runButtonCallback();
            }
        });

        String ip = getLocalIpAddress();
        if (ip != null) {
            myIP.setText(ip);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (status == Status.RUNNING) {
            new Thread(mainThread).start();
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        mainThread.stop();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mainThread.stop();
    }

    private void runButtonCallback() {
        String ipRegexExpression =
            "^(([01]?\\d\\d?|2[0-4]\\d|25[0-5])\\.){3}([01]?\\d\\d?|2[0-4]\\d|25[0-5])$";

        switch (status) {
            case WAITING:
                String sMasterIP = masterIP.getText().toString();
                if (! sMasterIP.matches(ipRegexExpression)) {
                    statusText.setText(R.string.status_masterIP_error);
                    break;
                }
                Log.i(TAG, "master ip is fine: " + sMasterIP);

                String sMyIP = myIP.getText().toString();
                if (! sMyIP.matches(ipRegexExpression)) {
                    statusText.setText(R.string.status_myIP_error);
                    break;
                }
                Log.i(TAG, "my ip is fine: " + sMyIP);

                if (! mainThread.checkRosMaster(sMasterIP, sMyIP)) {
                    statusText.setText(R.string.status_check_master_error);
                    break;
                }
                Log.i(TAG, "Master is ready");

                statusText.setText(R.string.status_running);
                runButton.setText(R.string.button_stop);

                new Thread(mainThread).start();
                status = Status.RUNNING;
                break;

            case RUNNING:
                statusText.setText(R.string.status_waiting);
                runButton.setText(R.string.button_run);

                mainThread.stop();
                status = Status.WAITING;
                break;
        }
    }

    private String getLocalIpAddress(){
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces();
            en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress() && inetAddress instanceof Inet4Address) {
                        return inetAddress.getHostAddress();
                    }
                }
            }
        } catch (Exception ex) {
            Log.e("IP Address", ex.toString());
        }
        return null;
    }
}
