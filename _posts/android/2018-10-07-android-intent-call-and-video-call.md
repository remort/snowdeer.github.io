---
layout: post
title: Intent를 이용해서 전화 및 화상 통화 호출하는 방법
category: Android
tag: [Android]
---
# Intent를 이용해서 전화 및 화상 통화 호출하는 방법

제조사마다 방식이나 특정 Flag 값이 다를 수 있습니다.

<pre class="prettyprint">
import android.Manifest.permission;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.EditText;
import android.widget.Toast;

public class MainActivity extends AppCompatActivity {

  EditText etPhoneNumber;
  static final int PERMISSION_REQUEST_CODE = 100;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    etPhoneNumber = findViewById(R.id.et_phone_number);

    findViewById(R.id.btn_call).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        Intent i = new Intent(Intent.ACTION_CALL);
        i.setData(Uri.parse("tel:" + getPhoneNumber()));
        i.putExtra("android.phone.extra.calltype", 0);
        startActivity(i);
      }
    });

    findViewById(R.id.btn_video_call).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        Intent i = new Intent(Intent.ACTION_CALL);
        i.setData(Uri.parse("tel:" + getPhoneNumber()));
        i.putExtra("videocall", true);
        startActivity(i);
      }
    });

    if (checkSelfPermission(permission.CALL_PHONE)
        != PackageManager.PERMISSION_GRANTED) {
      String[] permissions = new String[]{permission.CALL_PHONE};
      requestPermissions(permissions, PERMISSION_REQUEST_CODE);
    }
  }

  String getPhoneNumber() {
    return etPhoneNumber.getText().toString();
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);

    switch (requestCode) {
      case PERMISSION_REQUEST_CODE:
        if (grantResults.length > 0
            && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
          Toast.makeText(getApplicationContext(), "Permission 완료", Toast.LENGTH_SHORT).show();
        } else {
          Toast.makeText(getApplicationContext(), "Permission 실패", Toast.LENGTH_SHORT).show();
        }
        break;
    }
  }

}
</pre>