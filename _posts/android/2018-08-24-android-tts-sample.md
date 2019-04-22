---
layout: post
title: TTS(Text to Speech) 샘플
category: Android
tag: [Android]
---

## activity_main.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"
  tools:context=".MainActivity"&gt;

  &lt;EditText
    android:id="@+id/edittext"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:hint="TTS Text를 입력하세요."/&gt;

  &lt;Button
    android:id="@+id/run"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="실행"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## MainActivity.java

<pre class="prettyprint">
import android.speech.tts.TextToSpeech;
import android.speech.tts.UtteranceProgressListener;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.EditText;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements TextToSpeech.OnInitListener {

  EditText editText;

  TextToSpeech tts;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    editText = findViewById(R.id.edittext);
    findViewById(R.id.run).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        String text = editText.getText().toString();
        speakText(text);
      }
    });

    initTts();
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    tts.shutdown();
  }

  private void initTts() {
    tts = new TextToSpeech(getApplicationContext(), this);
    tts.setOnUtteranceProgressListener(new UtteranceProgressListener() {

      @Override
      public void onStart(String utteranceId) {
        Log.i("snowdeer", "[snowdeer] onStart: " + utteranceId);
      }

      @Override
      public void onDone(String utteranceId) {
        Log.i("snowdeer", "[snowdeer] onDone: " + utteranceId);
      }

      @Override
      public void onError(String utteranceId) {
        Log.i("snowdeer", "[snowdeer] onError: " + utteranceId);
      }
    });
  }

  @Override
  public void onInit(int status) {
    if (status == TextToSpeech.SUCCESS) {
      Log.i("snowdeer", "[snowdeer] TTS is ready.");
      tts.setLanguage(Locale.KOREA);
      tts.setPitch(1.0f);
      tts.setSpeechRate(1.0f);
    } else {
      Log.w("snowdeer", "[snowdeer] TTS is not ready !!");
    }
  }

  private void speakText(String text) {
    String utteranceId = "utteranceId";
    tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, utteranceId);
  }
}
</pre>