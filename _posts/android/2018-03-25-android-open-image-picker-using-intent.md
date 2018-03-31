---
layout: post
title: Intent를 이용해서 Image Picker 호출
category: Android
tag: [Android]
---
# Intent를 이용해서 Image Picker 호출

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

  private static final int REQUEST_GALLERY = 200;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    findViewById(R.id.btn_gallery).setOnClickListener(mOnClickListener);
  }

  private View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View v) {
      switch (v.getId()) {
        case R.id.btn_gallery:
          openGallery();
          break;
      }
    }
  };

  private void openGallery() {
    Intent intent = new Intent();
    intent.setType("image/*");
    intent.setAction(Intent.ACTION_GET_CONTENT);
    startActivityForResult(Intent.createChooser(intent, "Select Picture"), REQUEST_GALLERY);
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    super.onActivityResult(requestCode, resultCode, data);

    if (resultCode != RESULT_OK) {
      return;
    }

    switch (requestCode) {
      case REQUEST_GALLERY:
        Uri uri = data.getData();
        
        try {
          GlobalVariable.bitmap = MediaStore.Images.Media.getBitmap(getContentResolver(), uri);
        } catch (Exception e) {
          e.printStackTrace();
        }
        Intent intent = new Intent(MainActivity.this, ImageActivity.class);
        startActivity(intent);
        break;
    }
  }
}
</pre>