---
layout: post
title: Estimote Beacon 스캔
category: Android
tag: [android, estimote, beacon]
---

안드로이드에서 [Estimote Beacon](http://estimote.com/)을 스캔하는 간단한 코드입니다.
더 자세한 정보는 [여기](http://developer.estimote.com/)에서 확인하세요.

그리고 Estimote SDK는 [여기](https://github.com/Estimote/Android-SDK)(GitHub)에서 다운 받을 수 있습니다.

Android Studio 기준으로, 일단 build.gradle에 Estimote Beacon SDK에 대한 dependency를 추가해줍니다.

<pre class="prettyprint">
dependencies {
    compile fileTree(dir: 'libs', include: ['*.jar'])
    compile 'com.android.support:appcompat-v7:23.0.1'

    // 여기를 추가해줍니다.
    compile 'com.estimote:sdk:0.11.1@aar'
}
</pre>

그런 다음, 다음과 같은 코드로 간단한게 주위에 있는 Estimote Beacon들을 스캔할 수 있습니다.



<pre class="prettyprint" style="font-size:1.6em;">
package snowdeer.sample.estimote;

import android.graphics.Bitmap;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.EstimoteSDK;
import com.estimote.sdk.Nearable;
import com.estimote.sdk.SystemRequirementsChecker;
import com.estimote.sdk.connection.scanner.ConfigurableDevicesScanner;
import com.estimote.sdk.connection.scanner.DeviceType;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "snowdeer";
    private TextView mLogView;
    private StringBuilder mLogBuilder = new StringBuilder();
    private Handler mHandler = new Handler();

    private BeaconManager mBeaconManager;
    private ConfigurableDevicesScanner mDeviceScanner;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        findViewById(R.id.btn_start_scan).setOnClickListener(mOnClickListener);
        findViewById(R.id.btn_stop_scan).setOnClickListener(mOnClickListener);
        findViewById(R.id.btn_clear_log).setOnClickListener(mOnClickListener);

        mLogView = (TextView)findViewById(R.id.log);
        mLogBuilder.setLength(0);

        EstimoteSDK.initialize(getApplicationContext(), "estimotebeaconsample-ivb", "45f6fc31fd613624b0f24df41b121db6");

        mBeaconManager = new BeaconManager(getApplicationContext());
        mDeviceScanner = new ConfigurableDevicesScanner(getApplicationContext());
    }

    @Override
    protected void onResume() {
        super.onResume();
        SystemRequirementsChecker.checkWithDefaultDialogs(this);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    private void log(String message) {
        mLogBuilder.append("\n");
        mLogBuilder.append(message);
        mHandler.post(new Runnable() {
            @Override
            public void run() {
                if(mLogView != null) {
                    mLogView.setText(mLogBuilder.toString());
                }
            }
        });
    }

    private View.OnClickListener mOnClickListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            switch(view.getId()) {
                case R.id.btn_start_scan:
                    startToScan();
                    break;
                case R.id.btn_stop_scan:
                    stopToScan();
                    break;
                case R.id.btn_clear_log:
                    clearLog();
                    break;
            }
        }
    };

    private void startToScan() {
        log("start to scan beacons()");

        mDeviceScanner.setOwnDevicesFiltering(false);
        ArrayList&lt;DeviceType&gt; deviceTypeList = new ArrayList&lt;DeviceType&gt;();
        deviceTypeList.add(DeviceType.LOCATION_BEACON);
        deviceTypeList.add(DeviceType.NEARABLE);
        deviceTypeList.add(DeviceType.PROXIMITY_BEACON);
        mDeviceScanner.setDeviceTypes(deviceTypeList);
        mDeviceScanner.scanForDevices(new ConfigurableDevicesScanner.ScannerCallback() {

            @Override
            public void onDevicesFound(List&lt;ConfigurableDevicesScanner.ScanResultItem&gt; devices) {
                log("onDevicesFound : " + devices.size());
                for(int i=0; i<devices.size(); i++) {
                    ConfigurableDevicesScanner.ScanResultItem item = devices.get(i);
                    log((i+1) + ". Device ID : " + item.device.deviceId);
                    log("  - Device Type : " + item.device.type);
                    log("  - RSSI : " + item.rssi);
               }

                log("");
            }
        });
    }

    private void stopToScan() {
        log("stop to scan beacons()");
        mDeviceScanner.stopScanning();
    }

    private void clearLog() {
        mLogBuilder.setLength(0);
        mLogView.setText("");
    }
}

</pre>