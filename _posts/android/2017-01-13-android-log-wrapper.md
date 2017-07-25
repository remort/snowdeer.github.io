---
layout: post
title: Log Class
category: Android
tag: [Android]
---

안드로이드 Log 기능을 Wrapping한 클래스 코드입니다.
안드로이드에서 기본으로 제공하는 Log를 써도 되지만 나중에 일괄적으로 visible/hidden을
적용하거나 특수한 Prefix 등을 붙인다거나 등 좀 더 편리하게 쓰기 위해서는
별도로 Log 클래스를 두는 것이 편리합니다.

<br>
## Log 클래스 코드

<pre class="prettyprint">package snowdeer.util;

public class Log {

  private static final String TAG = "snowdeer";
  private static final String PREFIX = "[snowdeer] ";
  private static boolean useLog = true;

  public static void v(String tag, String message) {
    if(useLog) {
      android.util.Log.v(tag, PREFIX + message);
    }
  }

  public static void d(String tag, String message) {
    if(useLog) {
      android.util.Log.d(tag, PREFIX + message);
    }
  }

  public static void i(String tag, String message) {
    if(useLog) {
      android.util.Log.i(tag, PREFIX + message);
    }
  }

  public static void w(String tag, String message) {
    if(useLog) {
      android.util.Log.w(tag, PREFIX + message);
    }
  }

  public static void e(String tag, String message) {
    if(useLog) {
      android.util.Log.e(tag, PREFIX + message);
    }
  }

  public static void v(String message) {
    android.util.Log.v(TAG, message);
  }

  public static void d(String message) {
    android.util.Log.d(TAG, message);
  }

  public static void i(String message) {
    android.util.Log.i(TAG, message);
  }

  public static void w(String message) {
    android.util.Log.w(TAG, message);
  }

  public static void e(String message) {
    android.util.Log.e(TAG, message);
  }
}
</pre>
