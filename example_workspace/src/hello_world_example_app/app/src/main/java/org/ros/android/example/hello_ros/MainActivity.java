package org.ros.android.example.hello_ros;

import android.Manifest;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.lang.Runnable;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class MainActivity extends Activity {
    static {
        System.loadLibrary("native-activity");
    }

    private static final String IP_REGEX_EXPRESSION =
            "^(([01]?\\d\\d?|2[0-4]\\d|25[0-5])\\.){3}([01]?\\d\\d?|2[0-4]\\d|25[0-5])$";
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
        public native boolean checkRosMaster(List<Pair<String, String>> remappings);
        private native void __RosThread();
    }

    private RosThread mainThread;
    private EditText masterIP;
    private EditText myIP;
    private EditText remappingArgs;
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
        remappingArgs = (EditText) findViewById(R.id.remapping_args);
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
        switch (status) {
            case WAITING:
                String sMasterIP = masterIP.getText().toString();
                if (!sMasterIP.matches(IP_REGEX_EXPRESSION)) {
                    statusText.setText(R.string.status_masterIP_error);
                    break;
                }
                Log.i(TAG, "master ip is fine: " + sMasterIP);

                String sMyIP = myIP.getText().toString();
                if (!sMyIP.matches(IP_REGEX_EXPRESSION)) {
                    statusText.setText(R.string.status_myIP_error);
                    break;
                }
                Log.i(TAG, "my ip is fine: " + sMyIP);

                List<Pair<String, String>> remappings = new ArrayList();
                Pair<String, String> master = new Pair<String, String>("__master", "http://" + sMasterIP + ":11311");
                Pair<String, String> ip = new Pair<String, String>("__ip", sMyIP);
                remappings.add(master);
                remappings.add(ip);

                // Parse remappings - basic error handling.
                String sRemappingArgs = remappingArgs.getText().toString();
                if (!"".equals(sRemappingArgs)) {
                    try {
                        String[] remappingExtras = remappingArgs.getText().toString().split(" ");
                        for (String remapping : remappingExtras) {
                            String[] remappingBits = remapping.split(":=");
                            Pair<String, String> remappingPair = new Pair<String, String>(remappingBits[0], remappingBits[1]);
                            remappings.add(remappingPair);
                        }
                    } catch (Exception e) {
                        Log.e(TAG, "Could not parse remappings properly.", e);
                    }
                } else {
                    Log.i(TAG, "No remapping arguments provided.");
                }

                if (!mainThread.checkRosMaster(remappings)) {
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
