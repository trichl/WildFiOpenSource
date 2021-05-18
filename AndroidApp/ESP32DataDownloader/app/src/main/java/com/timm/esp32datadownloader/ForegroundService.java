package com.timm.esp32datadownloader;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;

import androidx.annotation.Nullable;
import androidx.core.app.NotificationCompat;

import java.io.IOException;

public class ForegroundService extends Service {
    public static final String CHANNEL_ID = "ForegroundServiceChannel";

    NanoHTTPDWebserver server;
    RawDataStorage rawDataStorage;
    LogDataStorage logDataStorage;

    PowerManager.WakeLock wakeLock;
    WifiManager.WifiLock wifiLock;

    @Override
    public void onCreate() {
        super.onCreate();
        rawDataStorage = new RawDataStorage(this);
        logDataStorage = new LogDataStorage(this);
        logDataStorage.writeToLog("Gateway started, storing to: " + rawDataStorage.getFileNameWithoutDirectory());
    }

    private void createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationChannel serviceChannel = new NotificationChannel(
                    CHANNEL_ID,
                    "Foreground Service Channel",
                    NotificationManager.IMPORTANCE_DEFAULT
            );
            NotificationManager manager = getSystemService(NotificationManager.class);
            manager.createNotificationChannel(serviceChannel);
        }
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        // Start the httpd.
        server = new NanoHTTPDWebserver(8080);
        server.setDataStorages(rawDataStorage, logDataStorage);
        try {
            server.start();
        } catch (IOException e) {
            logDataStorage.writeToLog("Error: could not start foreground service!");
        }
        // Acquire wake lock
        PowerManager powerManager = (PowerManager) getSystemService(POWER_SERVICE);
        wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "esp32datadownloader::nanohttpd");
        wakeLock.acquire();
        // Keep the wifi awake.
        WifiManager wm = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        wifiLock = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "esp32datadownloader::nanohttpdw");
        wifiLock.acquire();

        // Become a foreground service:
        String input = intent.getStringExtra("inputExtra");
        createNotificationChannel();
        Intent notificationIntent = new Intent(this, MainActivity.class);
        PendingIntent pendingIntent = PendingIntent.getActivity(this,
                0, notificationIntent, 0);
        Notification notification = new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle("Gateway running")
                .setContentText(input)
                .setSmallIcon(R.mipmap.notification)
                .setContentIntent(pendingIntent)
                .build();
        startForeground(1, notification);
        //do heavy work on a background thread
        //stopSelf();
        return Service.START_STICKY;
    }

    /*
    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        String input = intent.getStringExtra("inputExtra");
        createNotificationChannel();
        Intent notificationIntent = new Intent(this, MainActivity.class);
        PendingIntent pendingIntent = PendingIntent.getActivity(this,
                0, notificationIntent, 0);
        Notification notification = new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle("Foreground Service")
                .setContentText(input)
                .setSmallIcon(R.mipmap.notification)
                .setContentIntent(pendingIntent)
                .build();
        startForeground(1, notification);
        //do heavy work on a background thread
        //stopSelf();
        return Service.START_STICKY;
    }
    */

    @Override
    public void onDestroy() {
        //super.onDestroy();
        logDataStorage.writeToLog("Gateway stopped");
        stopForeground(true);
        if(wakeLock != null) wakeLock.release();
        if(wifiLock != null) wifiLock.release();
        server.stop();

    }
    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }
}