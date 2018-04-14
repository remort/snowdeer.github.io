---
layout: post
title: Runtime Permission
category: Android
tag: [Android, Permission]
---
# Permission

안드로이드에서 App이 단말의 특정 기능을 이용하려면 권한을 가져야 합니다. 특정 기능은 예로 들어
다음과 같은 기능들이 있습니다.

* 파일 읽고 쓰기
* 네트워크 사용, 인터넷 접속
* 카메라 사용
* 문자 메세지, 주소록 읽기 등

기존에는 `AndroidManifest.xml` 파일에 개발자가 사용하려는 권한을 등록하게 되어 있었고,
Google Play 등의 마켓에서 해당 App을 설치할 때 사용자가 권한 확인을 하도록 했습니다.
그리고 사용자가 원하지 않는 권한이 있을 경우, 그 App을 아예 이용할 수 없도록 했었습니다.

하지만, 6.0 M(마시멜로우) 버전부터는 권한 획득 방법이 바뀌었습니다. App이 실행할 때 사용자에게
권한을 요청하도록 되었으며, 사용자가 특정 권한을 원하지 않을 경우 해당 기능만 빼고 동작할 수 있도록
되었습니다. (물론, App 개발자가 사용자가 권한을 허가하지 않았을 때의 동작도 구현해놓아야 합니다.)

<br>

# 예제

다음 예제는 외부 저장소에 파일을 쓸 수 있는 권한을 요청하는 예제입니다.

<pre class="prettyprint">
  static final int PERMISSION_REQUEST_CODE = 100;

  protected void onCreate(Bundle savedInstanceState) {
      super.onCreate(savedInstanceState);
      setContentView(R.layout.activity_main);

      if (checkSelfPermission(permission.WRITE_EXTERNAL_STORAGE)
          != PackageManager.PERMISSION_GRANTED) {
        String[] permissions = new String[]{permission.WRITE_EXTERNAL_STORAGE};
        requestPermissions(permissions, PERMISSION_REQUEST_CODE);
      }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String permissions[],
      int[] grantResults) {
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
</pre>