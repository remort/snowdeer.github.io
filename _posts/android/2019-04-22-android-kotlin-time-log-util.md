---
layout: post
title: Kotlin TimeUtil 및 Log 클래스
category: Android
tag: [Android, Kotlin]
---

## TimeUtil.kt

<pre class="prettyprint">
import java.lang.Exception
import java.text.SimpleDateFormat
import java.util.*

class TimeUtil {
    companion object {

        @Synchronized
        fun getTimeAsLong(): Long {
            val calendar = Calendar.getInstance()
            return calendar.timeInMillis
        }

        @Synchronized
        fun getTimeAsString(format: String): String {
            val date = Date(getTimeAsLong())
            val sdf = SimpleDateFormat(format, Locale.getDefault())

            return sdf.format(date)
        }

        @Synchronized
        fun getTimeAsLong(format: String, text: String): Long {
            try {
                val sdf = SimpleDateFormat(format, Locale.getDefault())
                val date = sdf.parse(text)

                return date.time

            } catch (e: Exception) {
                e.printStackTrace()
            }

            return -1
        }

        @Synchronized
        fun getTimeAsString(format: String, time: Long): String {
            val date = Date(time)
            val sdf = SimpleDateFormat(format, Locale.getDefault())

            return sdf.format(date)
        }
    }
}
</pre>

<br>

## Log.kt

<pre class="prettyprint">
class Log {
    companion object {
        private const val TAG = "SampleApp"
        private const val PREFIX = "snowdeer"

        @Synchronized
        fun v(text: String) {
            android.util.Log.v(TAG, getDecoratedLog(text))
        }

        @Synchronized
        fun d(text: String) {
            android.util.Log.d(TAG, getDecoratedLog(text))
        }

        @Synchronized
        fun i(text: String) {
            android.util.Log.i(TAG, getDecoratedLog(text))
        }

        @Synchronized
        fun w(text: String) {
            android.util.Log.w(TAG, getDecoratedLog(text))
        }

        @Synchronized
        fun e(text: String) {
            android.util.Log.e(TAG, getDecoratedLog(text))
        }

        private fun getDecoratedLog(text: String): String {
            val sb = StringBuilder()

            sb.append("[$PREFIX] ")
            sb.append("[${TimeUtil.getTimeAsLong()}] ")
            sb.append(text)

            return sb.toString()
        }
    }
}
</pre>