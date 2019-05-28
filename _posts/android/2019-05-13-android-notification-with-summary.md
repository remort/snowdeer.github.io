---
layout: post
title: Summary를 지원하는 Notification 예제
category: Android
tag: [Android]
---

## NotificationHandler.kt

<pre class="prettyprint">
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.graphics.Color
import android.support.v4.app.NotificationCompat
import com.snowdeer.neverfi.R

class NotificationHandler {
    private lateinit var ctx: Context
    private lateinit var notificationManager: NotificationManager

    companion object {
        private const val CHANNEL_ID = "com.snowdeer.neverfi"
        private const val SUMMARY_NOTI_ID = 10003
        private const val NOTIFICATION_GROUP_KEY = "snowdeer_noti"
        val instance = NotificationHandler()
    }

    fun init(ctx: Context) {
        this.ctx = ctx
        notificationManager = ctx.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
    }

    fun createNotificationChannel(name: String, description: String) {
        val importance = NotificationManager.IMPORTANCE_LOW
        val channel = NotificationChannel(CHANNEL_ID, name, importance)

        channel.description = description
        channel.enableLights(true)
        channel.lightColor = Color.RED
        channel.enableVibration(true)
        channel.vibrationPattern = longArrayOf(100, 200, 300, 400, 500, 400, 300, 200, 400)
        notificationManager.createNotificationChannel(channel)
    }

    fun showNotification(id: Int, text: String, resultIntent: Intent) {
        val pendingIntent = PendingIntent.getActivity(ctx, 0, resultIntent, 0)
        val notification = Notification.Builder(ctx, CHANNEL_ID)
                .setContentText(text)
                .setSmallIcon(R.drawable.ic_launcher)
                .setChannelId(CHANNEL_ID)
                .setContentIntent(pendingIntent)
                .setGroup(NOTIFICATION_GROUP_KEY)
                .setGroupSummary(true)
                .build()

        notification.flags = Notification.FLAG_NO_CLEAR

        notificationManager.notify(id, notification)
        showSummaryNoti()
    }

    fun isPinnedItem(id: Int): Boolean {
        val notiList = notificationManager.activeNotifications

        for (n in notiList) {
            if (id == n.id) {
                return true
            }
        }

        return false
    }

    fun dismissNotification(id: Int) {

        notificationManager.cancel(id)
    }

    private fun showSummaryNoti() {
        val summaryNoti = NotificationCompat.Builder(ctx, CHANNEL_ID)
                .setSmallIcon(R.drawable.ic_launcher)
                .setStyle(NotificationCompat.InboxStyle()
                        .setSummaryText(ctx.getString(R.string.app_name)))
                .setGroup(NOTIFICATION_GROUP_KEY)
                .setGroupSummary(true)
                .build()

        notificationManager.notify(SUMMARY_NOTI_ID, summaryNoti)
    }
}
</pre>