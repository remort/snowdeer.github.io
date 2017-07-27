---
layout: post
title: URL을 이용해서 Bitmap 가져오기
category: Android
tag: [Android]
---

URL을 이용해서 HTTP 기반으로 이미지를 다운로드하는 코드입니다.

일단, 인터넷 경로를 통해 다운을 받기 위해서는 `AndroidManifest.xml` 파일에 
다음 permission을 추가해줘야 합니다.

<br>

# Permission

~~~
<uses-permission android:name="android.permission.INTERNET" />
~~~

<br>

# URL을 이용하여 Bitmap 가져오는 함수

<pre class="prettyprint">
private Bitmap getImageFromURL(String strImageURL) {
    Bitmap imgBitmap = null;

    try {
      URL url = new URL(strImageURL);
      URLConnection connection = url.openConnection();
      connection.connect();

      int size = conn.getContentLength();
      BufferedInputStream bis = new BufferedInputStream(connection.getInputStream(), size);
      imgBitmap = BitmapFactory.decodeStream(bis);

      bis.close();
    } catch (Exception e) {
      e.printStackTrace();
    }

    return imgBitmap;
  }
</pre>