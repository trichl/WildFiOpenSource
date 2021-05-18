package com.timm.esp32datadownloader;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.AppCompatButton;
import androidx.core.content.ContextCompat;

import android.app.ActivityManager;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.Uri;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.PowerManager;
import android.provider.Settings;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

class BlackforestTagCommand {
    String commandId;
    String commandValue;
}

class BlackforestTagCommandSetAccFrequency extends BlackforestTagCommand {
    public BlackforestTagCommandSetAccFrequency(int frequency) {
        commandId = "A";
        commandValue = "0";
    }
}

public class MainActivity extends AppCompatActivity {
    private static final String HOTSPOT_SSID = "mpidata";
    private static final String HOTSPOT_PASSWORD = "87654321";
    private static final String HOTSPOT_CHANNEL = "1, 6 or 11";

    private boolean serverRunning = false;

    Button bStartStop, bClearLog, bDataDecoder, bOpenFiles;
    TextView tStatus, tHeadline;
    RelativeLayout rLoadingPanel;
    LinearLayout lAlertHotspotOff, lAlertMobileInternetOn, lAlertHotspotOnButGatewayStopped, lAlertMemory;

    LogDataStorage logDataStorage;

    Thread thread;;

    /*private void checkPermissions() {
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if(this.checkSelfPermission(Manifest.permission.WRITE_SETTINGS) != PackageManager.PERMISSION_GRANTED) {
                if(!this.shouldShowRequestPermissionRationale(Manifest.permission.WRITE_SETTINGS)) {
                    final AlertDialog.Builder builder = new AlertDialog.Builder(this);
                    builder.setTitle("This app needs background location access");
                    builder.setMessage("Please grant location access so this app can detect beacons in the background.");
                    builder.setPositiveButton(android.R.string.ok, null);
                    builder.setOnDismissListener(new DialogInterface.OnDismissListener() {

                        @TargetApi(23)
                        @Override
                        public void onDismiss(DialogInterface dialog) {
                            requestPermissions(new String[]{Manifest.permission.WRITE_SETTINGS}, 0);
                        }

                    });
                    builder.show();
                }
                else {
                    final AlertDialog.Builder builder = new AlertDialog.Builder(this);
                    builder.setTitle("Functionality limited");
                    builder.setMessage("Since background location access has not been granted, this app will not be able to discover beacons in the background.  Please go to Settings -> Applications -> Permissions and grant background location access to this app.");
                    builder.setPositiveButton(android.R.string.ok, null);
                    builder.setOnDismissListener(new DialogInterface.OnDismissListener() {
                        @Override
                        public void onDismiss(DialogInterface dialog) {}
                    });
                    builder.show();
                }
            }
        }
    }*/

    public boolean checkIfHotspotEnabled() {
        int actualState = 0;
        WifiManager wifi = (WifiManager) getSystemService(this.getApplicationContext().WIFI_SERVICE);
        Method method = null;
        try {
            method = wifi.getClass().getDeclaredMethod("getWifiApState");
        } catch (NoSuchMethodException e) {
            return false;
        }
        method.setAccessible(true);
        try {
            actualState = (Integer) method.invoke(wifi, (Object[]) null);
        } catch (IllegalAccessException e) {
            return false;
        } catch (InvocationTargetException e) {
            return false;
        }
        if(actualState == 13) {
            return true;
        }
        return false;
    }

    public void showMessage(String headline, String text) {
        final AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(headline);
        builder.setMessage(text);
        builder.setPositiveButton(android.R.string.ok, null);
        builder.setOnDismissListener(new DialogInterface.OnDismissListener() {
            @Override
            public void onDismiss(DialogInterface dialog) {}
        });
        builder.show();
    }

    void setButtonStarted() {
        bStartStop.setText("GATEWAY RUNNING - CLICK TO STOP");
        if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.LOLLIPOP && bStartStop instanceof AppCompatButton) {
            // don't change color
        } else {
            bStartStop.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#FF80AB")));
        }
        rLoadingPanel.setVisibility(View.VISIBLE);
    }

    void setButtonStopped() {
        bStartStop.setText("START THE GATEWAY");
        if(Build.VERSION.SDK_INT <= Build.VERSION_CODES.LOLLIPOP && bStartStop instanceof AppCompatButton) {
            // don't change color
        } else {
            bStartStop.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#B9F6CA")));
        }
        rLoadingPanel.setVisibility(View.INVISIBLE);
    }

    /*void putIconInTaskbar() {
        NotificationCompat.Builder builder = new NotificationCompat.Builder(this, "ESP32DataDownloaderChannel")
                .setSmallIcon(R.mipmap.notification)
                .setContentTitle("Gateway running in background!")
                //.setContentText("")
                .setPriority(NotificationCompat.PRIORITY_DEFAULT)
                .setOngoing(true);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            int importance = NotificationManager.IMPORTANCE_DEFAULT;
            NotificationChannel channel = new NotificationChannel("ESP32DataDownloaderChannel", "ESP32DataDownloader", importance);
            channel.setDescription("Gateway running in background!");
            NotificationManager notificationManager = getSystemService(NotificationManager.class);
            notificationManager.createNotificationChannel(channel);
        }
        Intent intent = new Intent(this, MainActivity.class);
        intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
        PendingIntent pendingIntent = PendingIntent.getActivity(this, 0, intent, 0);

        NotificationManagerCompat notificationManager = NotificationManagerCompat.from(this);
        notificationManager.notify(123456799, builder.build());
    }

    public void removeIcon() {
        NotificationManager nMgr = (NotificationManager) this.getSystemService(Context.NOTIFICATION_SERVICE);
        nMgr.cancel(123456799);
    }*/

    /*public String getHotspotName3() { // NOT WORKING
        WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        Method[] methods = wifiManager.getClass().getDeclaredMethods();
        for (Method m: methods) {
            if(m.getName().equals("getWifiApConfiguration")) {
                WifiConfiguration config = null;
                try {
                    config = (WifiConfiguration)m.invoke(wifiManager);
                } catch (IllegalAccessException e) {
                    return "illegal access";
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                    return "invoc exception";
                }
                String ssid = config.SSID;
                String bssid = config.BSSID;
                return ssid;
            }
        }
        return "not found";
    }

    public String getHotspotName2() { // NOT WORKING
        WifiManager manager = (WifiManager) this.getSystemService(Context.WIFI_SERVICE);
        WifiConfiguration wifiConf;
        try {
            Method method = manager.getClass().getMethod("getWifiApConfiguration");
            wifiConf = (WifiConfiguration) method.invoke(manager);
            return wifiConf.SSID;
        } catch (Exception e) {
            return "";
        }
    }

    public static String getHotspotName(Context context) { // NOT WORKING
        WifiManager manager = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
        if(manager.isWifiEnabled()) {
            WifiInfo wifiInfo = manager.getConnectionInfo();
            if(wifiInfo != null) {
                NetworkInfo.DetailedState state = WifiInfo.getDetailedStateOf(wifiInfo.getSupplicantState());
                if(state == NetworkInfo.DetailedState.CONNECTED || state == NetworkInfo.DetailedState.OBTAINING_IPADDR) {
                    return wifiInfo.getSSID();
                }
                else {
                    return "netinfo missing: "+state.toString();
                }
            }
            else {
                return "wifi info null";
            }
        }
        return "not enabled";
    }*/

    @Override
    protected void onPause() {
        if(isMyServiceRunning(ForegroundService.class)) {
            Toast.makeText(this, "Gateway is still running in background!", Toast.LENGTH_LONG).show();
        }
        super.onPause();
    }

    @Override
    public void onBackPressed() {
        if(isMyServiceRunning(ForegroundService.class)) {
            Toast.makeText(this, "Gateway is still running, please stop first!", Toast.LENGTH_LONG).show();
        }
        else super.onBackPressed();
    }

    @Override
    protected void onResume() {
        if(!checkIfHotspotEnabled()) {
            showMessage("Please activate Hotspot", "Please go into your settings and activate your phone hotspot with name = " + HOTSPOT_SSID + " and password = " + HOTSPOT_PASSWORD + " on (IMPORTANT!) channel " + HOTSPOT_CHANNEL + ".");
        }
        /*if(isInternetActivated()) {
            showMessage("Please deactivate your mobile Internet", "Please go into your settings and deactivate your mobile Internet. Otherwise the gateway won't work correctly.");
        }*/
        super.onResume();
    }

    public void startService() {
        Intent serviceIntent = new Intent(this, ForegroundService.class);
        serviceIntent.putExtra("inputExtra", "Gateway running in background");
        ContextCompat.startForegroundService(this, serviceIntent);
    }

    public void stopService() {
        Intent serviceIntent = new Intent(this, ForegroundService.class);
        stopService(serviceIntent);
    }

    private boolean isMyServiceRunning(Class<?> serviceClass) {
        ActivityManager manager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
        for (ActivityManager.RunningServiceInfo service : manager.getRunningServices(Integer.MAX_VALUE)) {
            if (serviceClass.getName().equals(service.service.getClassName())) {
                return true;
            }
        }
        return false;
    }

    private boolean isInternetActivated() {
        ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo activeNetwork = cm.getActiveNetworkInfo();
        boolean isConnected = activeNetwork != null && activeNetwork.isConnectedOrConnecting();
        return isConnected;
    }

    public static void noBatteryOptimization(Context context){
        Intent intent = new Intent();
        String packageName = context.getPackageName();
        PowerManager pm = (PowerManager) context.getSystemService(POWER_SERVICE);
        if(!pm.isIgnoringBatteryOptimizations(packageName)) {
            intent.setAction(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS);
            intent.setData(Uri.parse("package:" + "com.timm.esp32datadownloader"));
            context.startActivity(intent);
        }
    }

    public void updateHeadlineText() {
        if(tHeadline != null)
            tHeadline.setText("Free memory: " + StorageGeneral.getAvailableInternalMemorySizeAsStringInMByte() + " MByte\n(min. " + (StorageGeneral.getMinimumMemory() / (1024 * 1024)) + " MByte)");
    }

    private void setAlertWindows() {
        if(!checkIfHotspotEnabled()) lAlertHotspotOff.setVisibility(View.VISIBLE);
        else lAlertHotspotOff.setVisibility(View.GONE);

        if(isInternetActivated()) lAlertMobileInternetOn.setVisibility(View.VISIBLE);
        else lAlertMobileInternetOn.setVisibility(View.GONE);

        if(checkIfHotspotEnabled() && !isMyServiceRunning(ForegroundService.class)) lAlertHotspotOnButGatewayStopped.setVisibility(View.VISIBLE);
        else lAlertHotspotOnButGatewayStopped.setVisibility(View.GONE);

        if((StorageGeneral.getAvailableInternalMemorySize() / (1024 * 1024)) < 256) lAlertMemory.setVisibility(View.VISIBLE);
        else lAlertMemory.setVisibility(View.GONE);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.getSupportActionBar().hide();
        setContentView(R.layout.activity_main);

        logDataStorage = new LogDataStorage(this);

        bStartStop = (Button) findViewById(R.id.bstartstop);
        bClearLog = (Button) findViewById(R.id.bclearlog);
        bDataDecoder = (Button) findViewById(R.id.bdatadecoder);
        bOpenFiles = (Button) findViewById(R.id.bopenfiles);
        tStatus = (TextView) findViewById(R.id.tstatus);
        rLoadingPanel = (RelativeLayout) findViewById(R.id.rloadingpanel);
        rLoadingPanel.setVisibility(View.INVISIBLE);
        tHeadline = (TextView) findViewById(R.id.theadline);

        lAlertHotspotOff = findViewById(R.id.lalerthotspotoff);
        lAlertMobileInternetOn = findViewById(R.id.lalertmobileinterneton);
        lAlertHotspotOnButGatewayStopped = findViewById(R.id.lalerthotspotonbutgatewaystopped);
        lAlertMemory = findViewById(R.id.lalertmemory);

        setAlertWindows();

        // Start service
        if(isMyServiceRunning(ForegroundService.class)) {
            setButtonStarted();
        } else {
            setButtonStopped();
        }

        tStatus.setText(logDataStorage.readLastLogs(150));
        updateHeadlineText();

        thread = new Thread() {
            @Override
            public void run() {
                try {
                    while(!thread.isInterrupted()) {
                        Thread.sleep(3000);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                tStatus.setText(logDataStorage.readLastLogs(100));
                                updateHeadlineText();
                                setAlertWindows();
                            }
                        });
                    }
                } catch (InterruptedException e) {}
            }
        };
        thread.start();

        bStartStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(isMyServiceRunning(ForegroundService.class)) {
                    System.out.println("STOPPING SERVER");
                    stopService();
                    setButtonStopped();
                }
                else {
                    System.out.println("STARTING SERVER AGAIN");
                    if(!checkIfHotspotEnabled()) {
                        showMessage("Please activate Hotspot", "Please go into your settings and activate your phone hotspot with name = " + HOTSPOT_SSID + " and password = " + HOTSPOT_PASSWORD + " on (IMPORTANT!) channel " + HOTSPOT_CHANNEL + ".");
                    }
                    // No battery optimization
                    noBatteryOptimization(MainActivity.this);
                    startService();
                    setButtonStarted();
                }
            }
        });

        bClearLog.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                tStatus.setText("");
                logDataStorage.deleteLogFile();
            }
        });

        bDataDecoder.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            Intent intent = new Intent(MainActivity.this, DecoderActivity.class);
            //intent.putExtra("filename", files[selectedItem].getName());
            MainActivity.this.startActivity(intent);
            }
        });

        bOpenFiles.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Uri selectedUri = Uri.parse(MainActivity.this.getExternalFilesDir(null).getPath());
                Intent intent = new Intent(Intent.ACTION_VIEW);
                intent.setDataAndType(selectedUri, "resource/folder");
                if(intent.resolveActivityInfo(getPackageManager(), 0) != null) {
                    startActivity(intent);
                }
                else {
                    Toast.makeText(MainActivity.this, "No app found to view folders, please download a file manager", Toast.LENGTH_LONG).show();
                }
            }
        });
    }
}
