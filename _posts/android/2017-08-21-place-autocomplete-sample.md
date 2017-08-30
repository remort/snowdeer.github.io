---
layout: post
title: Place Picker 예제
category: Android
tag: [Android, Location]
---
Place Picker를 사용하기 위해서는 [Google API Key를 등록](/android/2017/01/23/android-how-to-use-google-api-key/)해야 합니다. 
Google 라이브러리들 중 'Google Places API for Android'를 등록하면 됩니다.

<br>

# 예제 코드

<pre class="prettyprint">
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Toast;
import com.google.android.gms.location.places.Place;
import com.google.android.gms.location.places.ui.PlacePicker;

public class MainActivity extends AppCompatActivity {

  int PLACE_PICKER_REQUEST = 1;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    findViewById(R.id.btn_place_picker).setOnClickListener(mOnClickListener);
  }

  View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View v) {
      switch (v.getId()) {
        case R.id.btn_place_picker:
          callPlacePicker();
          break;
      }
    }
  };

  void callPlacePicker() {
    PlacePicker.IntentBuilder builder = new PlacePicker.IntentBuilder();
    try {
      startActivityForResult(builder.build(this), PLACE_PICKER_REQUEST);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    if (requestCode == PLACE_PICKER_REQUEST) {
      if (resultCode == RESULT_OK) {
        Place place = PlacePicker.getPlace(data, this);
        String toastMsg = String.format("Place: %s", place.getName());
        Toast.makeText(this, toastMsg, Toast.LENGTH_LONG).show();
      }
    }
  }
}
</pre>