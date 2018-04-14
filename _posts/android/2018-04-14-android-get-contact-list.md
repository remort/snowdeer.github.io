---
layout: post
title: 연락처 정보 가져오기
category: Android
tag: [Android]
---
# 연락처 정보 가져오기

안드로이드내의 연락처 정보를 리스트 형태로 가져오는 코드입니다.

<br>

## AndroidManifest.xml

`AndroidManifest.xml` 파일에 다음 권한을 추가합니다.

<pre class="prettyprint">
&lt;uses-permission android:name="android.permission.READ_CONTACTS" tools:remove="android:maxSdkVersion"/&gt;
</pre>

<br>

## Java 코드

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

  static final int PERMISSION_REQUEST_CODE = 100;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    if (checkSelfPermission(permission.READ_CONTACTS)
        != PackageManager.PERMISSION_GRANTED) {
      String[] permissions = new String[]{permission.READ_CONTACTS};
      requestPermissions(permissions, PERMISSION_REQUEST_CODE);
    }

    findViewById(R.id.btn_get_contact_list).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        getContactListAsLog();
      }
    });
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String permissions[],
      int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);

    switch (requestCode) {
      case PERMISSION_REQUEST_CODE: {
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

  void getContactListAsLog() {
    ArrayList&lt;ContactInfo&gt; list = getContactList();

    for (ContactInfo info : list) {
      Log.i("", info.toString());
    }
  }

  ArrayList&lt;ContactInfo&gt; getContactList() {
    ArrayList&lt;ContactInfo&gt; list = new ArrayList<>();

    Uri uri = Phone.CONTENT_URI;
    String[] projection = new String[]{
        Contacts._ID,
        Contacts.PHOTO_ID,
        Phone.DISPLAY_NAME,
        Phone.NUMBER,
    };

    String sortOrder = Phone.DISPLAY_NAME + " COLLATE LOCALIZED ASC";

    Cursor cursor = getContentResolver().query(uri, projection, null, null, sortOrder);
    if (cursor.moveToFirst()) {
      do {
        ContactInfo info = new ContactInfo();

        info.id = cursor.getLong(0);
        info.photoId = cursor.getLong(1);
        info.displayName = cursor.getString(2);
        info.phoneNumber = cursor.getString(3);
        list.add(info);

      } while (cursor.moveToNext());
    }

    return list;
  }

  private Bitmap getContactPicture(long contactId) {

    Bitmap bm = null;

    try {
      InputStream inputStream = ContactsContract.Contacts
          .openContactPhotoInputStream(getContentResolver(),
              ContentUris.withAppendedId(ContactsContract.Contacts.CONTENT_URI, contactId));

      if (inputStream != null) {
        bm = BitmapFactory.decodeStream(inputStream);
        inputStream.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    return bm;
  }

  class ContactInfo {

    long id;
    String displayName;
    String phoneNumber;
    long photoId;

    @Override
    public String toString() {
      return "ContactInfo{" +
          "id=" + id +
          ", displayName='" + displayName + '\'' +
          ", phoneNumber='" + phoneNumber + '\'' +
          ", photoId=" + photoId +
          '}';
    }
  }
}
</pre>