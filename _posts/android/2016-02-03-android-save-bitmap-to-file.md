---
layout: post
title: Bitmap 이미지를 파일로 저장하는 방법
category: Android
tag: [Android]
---

Bitmap 이미지를 파일로 저장하는 코드는 다음과 같습니다.

<br>

<pre class="prettyprint">
private void saveBitmapAsFile(Bitmap bitmap, String filepath) {
    File file = new File(filepath);
    OutputStream os = null;

    try {
      file.createNewFile();
      os = new FileOutputStream(file);

      bitmap.compress(CompressFormat.PNG, 100, os);

      os.close();
    } catch (Exception e) {
      e.printStackTrace();
    } 
  }
</pre>
