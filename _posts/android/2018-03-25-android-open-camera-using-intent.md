---
layout: post
title: Intent를 이용해서 시스템 카메라(Camera) 요청하기
category: Android
tag: [Android]
---
# Intent를 이용해서 Camera 사용하기

Intent를 이용해서 Camera 기능을 요청하는 코드입니다.

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

  private static final int REQUEST_CAMERA = 100;
  
  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    findViewById(R.id.btn_camera).setOnClickListener(mOnClickListener);
  }

  private View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View v) {
      switch (v.getId()) {
        case R.id.btn_camera:
          launchCameraActivity();
          break;
      }
    }
  };

  private void launchCameraActivity() {
    Intent intent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
    startActivityForResult(intent, REQUEST_CAMERA);
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    super.onActivityResult(requestCode, resultCode, data);

    if (resultCode != RESULT_OK) {
      return;
    }

    switch (requestCode) {
      case REQUEST_CAMERA:
        Log.i("", "[snowdeer] REQUEST_CAMERA !!");
        Bundle extras = data.getExtras();
        GlobalVariable.bitmap = (Bitmap) extras.get("data");

        Intent intent = new Intent(MainActivity.this, ImageActivity.class);
        startActivity(intent);
        break;
      
    }
  }
}
</pre>