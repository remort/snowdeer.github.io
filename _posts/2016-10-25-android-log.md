---
layout: post
title: Log Class
category: Android
tag: [android, log]
---

안드로이드 Log 기능을 Wrapping한 클래스입니다.

<pre class="prettyprint" style="font-size:0.7em;">
package snowdeer.utils;

import android.util.Log;

public class SnowLog {
    private static final String TAG = "snowdeer.app";
    private static final String PREFIX = "[snowdeer] ";
    private static boolean useLog = true;

    public static void v(String tag, String message) {
        if(useLog) {
            Log.v(tag, PREFIX + message);
        }
    }

    public static void d(String tag, String message) {
        if(useLog) {
            Log.d(tag, PREFIX + message);
        }
    }

    public static void i(String tag, String message) {
        if(useLog) {
            Log.i(tag, PREFIX + message);
        }
    }

    public static void w(String tag, String message) {
        if(useLog) {
            Log.w(tag, PREFIX + message);
        }
    }

    public static void e(String tag, String message) {
        if(useLog) {
            Log.e(tag, PREFIX + message);
        }
    }

    public static void v( String message) {
        Log.v(TAG, message);
    }

    public static void d( String message) {
        Log.d(TAG, message);
    }

    public static void i( String message) {
        Log.i(TAG, message);
    }

    public static void w( String message) {
        Log.w(TAG, message);
    }

    public static void e( String message) {
        Log.e(TAG, message);
    }
}

</pre>
