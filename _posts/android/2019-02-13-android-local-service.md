---
layout: post
title: Local Service 예제
category: Android
tag: [Android]
---

## SnowService.java

<pre class="prettyprint">
package com.snowdeer.local_service;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

public class SnowService extends Service {

  int num = 0;

  public int getNumber() {
    num++;
    return num;
  }

  public class LocalBinder extends Binder {
    SnowService getService() {
      return SnowService.this;
    }
  }

  final Binder binder = new LocalBinder();

  @Override
  public IBinder onBind(Intent intent) {
    Log.i("snowdeer", "[snowdeer] SnowService - onBind()");
    return binder;
  }

  @Override
  public boolean onUnbind(Intent intent) {
    Log.i("snowdeer", "[snowdeer] SnowService - onUnbind()");
    return super.onUnbind(intent);
  }
}
</pre>

<br>

## MainActivity.java

<pre class="prettyprint">
package com.snowdeer.local_service;

import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import com.snowdeer.local_service.SnowService.LocalBinder;

public class MainActivity extends AppCompatActivity {

  SnowService snowService = null;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    Log.i("snowdeer", "[snowdeer] onCreate");

    Intent i = new Intent(MainActivity.this, SnowService.class);
    startService(i);

    bindService(i, mConnection, BIND_AUTO_CREATE);
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    unbindService(mConnection);
    Log.i("snowdeer", "[snowdeer] onDestroy");
  }

  ServiceConnection mConnection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      Log.i("snowdeer", "[snowdeer] onServiceConnected");

      snowService = ((LocalBinder)service).getService();

      Log.i("snowdeer", "[snowdeer] num: " + snowService.getNumber());
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      Log.i("snowdeer", "[snowdeer] onServiceDisconnected");
    }
  };
}
</pre>