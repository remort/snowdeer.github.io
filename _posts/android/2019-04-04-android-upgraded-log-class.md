---
layout: post
title: 클래스명, 함수명까지 자동으로 출력해주는 Log class
category: Android
tag: [Android]
---

## TimeUtil.java

<pre class="prettyprint">
import android.icu.util.Calendar;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class TimeUtil {

  public synchronized static long getTimeAsLong() {
    Calendar calendar = Calendar.getInstance();
    return calendar.getTimeInMillis();
  }

  public synchronized static String getTimeAsString(String format) {
    Date date = new Date(getTimeAsLong());
    SimpleDateFormat sdf = new SimpleDateFormat(format,
        Locale.getDefault());

    return sdf.format(date);
  }

  public synchronized static Long getTimeAsLong(String format, String text) {
    try {
      SimpleDateFormat sdf = new SimpleDateFormat(format,
          Locale.getDefault());
      Date date = sdf.parse(text);
      return date.getTime();
    } catch (Exception e) {
      e.printStackTrace();
    }
    return Long.valueOf(-1);
  }

  public synchronized static String getTimeAsString(String format, long time) {
    Date date = new Date(time);
    SimpleDateFormat sdf = new SimpleDateFormat(format,
        Locale.getDefault());

    return sdf.format(date);
  }
}
</pre>

<br>

## Log.java

<pre class="prettyprint">
public class Log {

  private static final String TAG = "ActionManager";
  private static final String PREFIX = "[ram]";

  public synchronized static void v(String message) {
    android.util.Log.v(TAG, getDecoratedLog(message));
  }

  public synchronized static void d(String message) {
    android.util.Log.d(TAG, getDecoratedLog(message));
  }

  public synchronized static void i(String message) {
    android.util.Log.i(TAG, getDecoratedLog(message));
  }

  public synchronized static void w(String message) {
    android.util.Log.w(TAG, getDecoratedLog(message));
  }

  public synchronized static void e(String message) {
    android.util.Log.e(TAG, getDecoratedLog(message));
  }

  private static String getDecoratedLog(String message) {
    StackTraceElement ste = Thread.currentThread().getStackTrace()[4];

    StringBuilder sb = new StringBuilder();
    sb.append(PREFIX);
    sb.append(" [");
    sb.append(TimeUtil.getTimeAsLong());
    sb.append(" ");
    sb.append(ste.getFileName().replace(".java", ""));
    sb.append("::");
    sb.append(ste.getMethodName());
    sb.append("] ");
    sb.append(message);
    return sb.toString();
  }
}
</pre>

만약 Line Number까지 출력하고 싶으면 `ste` 변수의 `getLineNumber()` 메소드를 사용하면 됩니다.