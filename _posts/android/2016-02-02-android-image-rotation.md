---
layout: post
title: 이미지의 Rotation 정보를 획득하고 회전하는 방법
category: Android
tag: [Android]
---

안드로이드에서 사진을 ImageView 등에 출력할 때, 이미지의 Rotation 정보를 획득하여 적절하게 회전시켜주는 방법입니다.

<br>

# 이미지의 Orientation 정보 획득

<pre class="prettyprint">
public int getOrientationOfImage(String filepath) {
    ExifInterface exif = null;

    try {
      exif = new ExifInterface(filepath);
    } catch (IOException e) {
      e.printStackTrace();
      return -1;
    }

    int orientation = exif.getAttributeInt(ExifInterface.TAG_ORIENTATION, -1);

    if (orientation != -1) {
      switch (orientation) {
        case ExifInterface.ORIENTATION_ROTATE_90:
          return 90;

        case ExifInterface.ORIENTATION_ROTATE_180:
          return 180;

        case ExifInterface.ORIENTATION_ROTATE_270:
          return 270;
      }
    }

    return 0;
  }
</pre>

<br>

# Image 회전

<pre class="prettyprint">
  public Bitmap getRotatedBitmap(Bitmap bitmap, int degrees) throws Exception {
    if(bitmap == null) return null;
    if (degrees == 0) return bitmap;
    
    Matrix m = new Matrix();
    m.setRotate(degrees, (float) bitmap.getWidth() / 2, (float) bitmap.getHeight() / 2);

    return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), m, true);;
  }
</pre>