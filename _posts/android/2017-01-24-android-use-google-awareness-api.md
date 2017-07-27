---
layout: post
title: Google Awareness API 예제
category: Android
tag: [Android, Google API]
---

Google Awareness API 사용 예제입니다. 일단 앞서 포스팅한
[Google API Key 등록]({{ site.baseurl }}/android/2017/01/23/android-how-to-use-google-api-key/)을
먼저 해야 합니다. Google API Key를 획득했다면 다음과 같이 `AndroidManifest.xml` 에 해당 키 정보를 기입해줍니다.

<br>

# AndroidManifest.xml

<pre class="prettyprint">&lt;application
  android:allowBackup="true"
  android:icon="@mipmap/ic_launcher"
  android:label="@string/app_name"
  android:supportsRtl="true"
  android:theme="@style/AppTheme"&gt;

  &lt;meta-data
    android:name="com.google.android.awareness.API_KEY"
    android:value="API_KEY" /&gt;

  &lt;activity android:name=".MainActivity"&gt;
    &lt;intent-filter&gt;
      &lt;action android:name="android.intent.action.MAIN" /&gt;

      &lt;category android:name="android.intent.category.LAUNCHER" /&gt;
    &lt;/intent-filter&gt;
  &lt;/activity&gt;
&lt;/application&gt;</pre>
<br>

# Activity_main.xml

그리고 간단한 버튼 4개를 배치해서 테스트를 하기 위한 activity_main.xml 코드는 다음과 같습니다.
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:id="@+id/activity_main"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:paddingTop="@dimen/activity_vertical_margin"
  android:paddingBottom="@dimen/activity_vertical_margin"
  android:paddingLeft="@dimen/activity_horizontal_margin"
  android:paddingRight="@dimen/activity_horizontal_margin"
  android:orientation="vertical"&gt;

  &lt;Button
    android:id="@+id/btn_get_location"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Get Location" /&gt;

  &lt;Button
    android:id="@+id/btn_get_activity"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Get Activity" /&gt;

  &lt;Button
    android:id="@+id/btn_get_place"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Get Place" /&gt;

  &lt;Button
    android:id="@+id/btn_get_weather"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Get Weather" /&gt;

  &lt;ScrollView
    android:layout_width="match_parent"
    android:layout_height="match_parent"&gt;

    &lt;TextView
      android:id="@+id/textview"
      android:layout_width="match_parent"
      android:layout_height="match_parent" /&gt;

  &lt;/ScrollView&gt;
&lt;/LinearLayout&gt;</pre>
<br>

# MainActivity.java

마지막으로, MainActivity.java 코드입니다.
<pre class="prettyprint">package com.datacafe.googleawarenessapi;

import android.content.pm.PackageManager;
import android.location.Location;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import com.google.android.gms.awareness.Awareness;
import com.google.android.gms.awareness.snapshot.DetectedActivityResult;
import com.google.android.gms.awareness.snapshot.LocationResult;
import com.google.android.gms.awareness.snapshot.PlacesResult;
import com.google.android.gms.awareness.snapshot.WeatherResult;
import com.google.android.gms.awareness.state.Weather;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.ResultCallback;
import com.google.android.gms.location.ActivityRecognitionResult;
import com.google.android.gms.location.DetectedActivity;
import com.google.android.gms.location.places.PlaceLikelihood;

import java.util.List;

public class MainActivity extends AppCompatActivity {

  //Activities
  //public static final int IN_VEHICLE = 0;
  //public static final int ON_BICYCLE = 1;
  //public static final int ON_FOOT = 2;
  //public static final int STILL = 3;
  //public static final int UNKNOWN = 4;
  //public static final int TILTING = 5;
  //public static final int WALKING = 7;
  //public static final int RUNNING = 8;

  private GoogleApiClient mGoogleApiClient;
  private TextView mDebugLogView;

  private StringBuilder mLogBuilder = new StringBuilder();

  private void log(String message) {
    mLogBuilder.append(message);
    mLogBuilder.append("\n");

    if(mDebugLogView != null) {
      mDebugLogView.setText(mLogBuilder.toString());
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    mDebugLogView = (TextView) findViewById(R.id.textview);

    findViewById(R.id.btn_get_location).setOnClickListener(mOnClickListener);
    findViewById(R.id.btn_get_activity).setOnClickListener(mOnClickListener);
    findViewById(R.id.btn_get_place).setOnClickListener(mOnClickListener);
    findViewById(R.id.btn_get_weather).setOnClickListener(mOnClickListener);

    Log.i("snowdeer", "[snowdeer] start...");

    mGoogleApiClient = new GoogleApiClient.Builder(this)
        .addApi(Awareness.API)
        .build();
    mGoogleApiClient.connect();

    Log.i("snowdeer", "[snowdeer] GoogleApi connect..");


  }

  private View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View view) {
      switch(view.getId()) {
        case R.id.btn_get_location: {
          if(ContextCompat.checkSelfPermission(
              MainActivity.this,
              android.Manifest.permission.ACCESS_FINE_LOCATION) !=
              PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(
                MainActivity.this,
                new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                100
            );
            return;
          }

          Awareness.SnapshotApi.getLocation(mGoogleApiClient)
              .setResultCallback(new ResultCallback&lt;LocationResult&gt;() {
                @Override
                public void onResult(@NonNull LocationResult locationResult) {
                  if(!locationResult.getStatus().isSuccess()) {
                    log("Could not get location.");
                    return;
                  }
                  Location location = locationResult.getLocation();
                  log("Lat: " + location.getLatitude()
                      + ", Lon: " + location.getLongitude());
                }
              });
          break;
        }
        case R.id.btn_get_activity:
          Awareness.SnapshotApi.getDetectedActivity(mGoogleApiClient)
              .setResultCallback(new ResultCallback&lt;DetectedActivityResult&gt;() {
                @Override
                public void onResult(@NonNull DetectedActivityResult detectedActivityResult) {
                  if(!detectedActivityResult.getStatus().isSuccess()) {
                    log("Could not get the current activity.");
                    return;
                  }
                  ActivityRecognitionResult ar = detectedActivityResult
                      .getActivityRecognitionResult();
                  DetectedActivity probableActivity = ar.getMostProbableActivity();
                  log(probableActivity.toString());
                }
              });
          break;

        case R.id.btn_get_place: {
          if(ContextCompat.checkSelfPermission(
              MainActivity.this,
              android.Manifest.permission.ACCESS_FINE_LOCATION) !=
              PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(
                MainActivity.this,
                new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                100
            );
            return;
          }

          Awareness.SnapshotApi.getPlaces(mGoogleApiClient)
              .setResultCallback(new ResultCallback&lt;PlacesResult&gt;() {
                @Override
                public void onResult(@NonNull PlacesResult placesResult) {
                  if(!placesResult.getStatus().isSuccess()) {
                    log("Could not get places.");
                    return;
                  }
                  List&lt;PlaceLikelihood&gt; placeLikelihoodList = placesResult.getPlaceLikelihoods();

                  if(placeLikelihoodList == null) {
                    log("Result List is Null!!");
                    return;
                  }
                  // Show the top 5 possible location results.
                  for(int i = 0; i &lt; placeLikelihoodList.size(); i++) {
                    PlaceLikelihood p = placeLikelihoodList.get(i);
                    log(p.getPlace().getName().toString()
                        + ", likelihood: " + p.getLikelihood());
                  }
                }
              });
          break;
        }

        case R.id.btn_get_weather: {
          if(ContextCompat.checkSelfPermission(
              MainActivity.this,
              android.Manifest.permission.ACCESS_FINE_LOCATION) !=
              PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(
                MainActivity.this,
                new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                100
            );
            return;
          }

          Awareness.SnapshotApi.getWeather(mGoogleApiClient)
              .setResultCallback(new ResultCallback&lt;WeatherResult&gt;() {
                @Override
                public void onResult(@NonNull WeatherResult weatherResult) {
                  if(!weatherResult.getStatus().isSuccess()) {
                    log("Could not get weather.");
                    return;
                  }
                  Weather weather = weatherResult.getWeather();
                  log("Weather: " + weather);
                }
              });

          break;
        }
      }
    }
  };
}</pre>
