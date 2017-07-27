---
layout: post
title: Notification 정보 가져오기
category: Android
tag: [Android]
---

안드로이드 노티피케이션(Notification)의 정보를 가져오는 코드를 작성해보도록 하겠습니다.
안드로이드에서 NotificationListenerService라는 서비스 형태의 컴포넌트를 제공하고 있습니다.
API 버전 18부터 사용가능하며, 원할하게 쓰려면 API 버전 19 이상을 추천합니다.

안드로이드 SDK에 기본으로 탑재되어 있기 때문에 gradle 등에 별도로 추가할 필요는 없고,
`manifest.xml`에 다음과 같이 서비스를 등록해주면 됩니다.

<br>

# AndroidManifest.xml

<pre class="prettyprint">&lt;application
  android:allowBackup="true"
  android:icon="@mipmap/ic_launcher"
  android:label="@string/app_name"
  android:supportsRtl="true"
  android:theme="@style/AppTheme"&gt;
  ...
  &lt;service
    android:name=".SnowNotificationListenerService"
    android:permission="android.permission.BIND_NOTIFICATION_LISTENER_SERVICE"&gt;
    &lt;intent-filter&gt;
      &lt;action android:name="android.service.notification.NotificationListenerService"/&gt;
    &lt;/intent-filter&gt;
  &lt;/service&gt;
&lt;/application&gt;</pre>
<br>

그리고, Java 코드는 다음과 같습니다.

# SnowNotificationListenerService.java

<pre class="prettyprint">import android.app.Notification;
import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.service.notification.NotificationListenerService;
import android.service.notification.StatusBarNotification;
import android.util.Log;

public class SnowNotificationListenerService extends NotificationListenerService {

  @Override
  public void onCreate() {
    super.onCreate();
    Log.i("NotificationListener", "[snowdeer] onCreate()");
  }

  @Override
  public int onStartCommand(Intent intent, int flags, int startId) {
    Log.i("NotificationListener", "[snowdeer] onStartCommand()");
    return super.onStartCommand(intent, flags, startId);
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
    Log.i("NotificationListener", "[snowdeer] onDestroy()");
  }

  @Override
  public void onNotificationPosted(StatusBarNotification sbn) {
    Log.i("NotificationListener", "[snowdeer] onNotificationPosted() - " + sbn.toString());
    Log.i("NotificationListener", "[snowdeer] PackageName:" + sbn.getPackageName());
    Log.i("NotificationListener", "[snowdeer] PostTime:" + sbn.getPostTime());

    Notification notificatin = sbn.getNotification();
    Bundle extras = notificatin.extras;
    String title = extras.getString(Notification.EXTRA_TITLE);
    int smallIconRes = extras.getInt(Notification.EXTRA_SMALL_ICON);
    Bitmap largeIcon = ((Bitmap) extras.getParcelable(Notification.EXTRA_LARGE_ICON));
    CharSequence text = extras.getCharSequence(Notification.EXTRA_TEXT);
    CharSequence subText = extras.getCharSequence(Notification.EXTRA_SUB_TEXT);

    Log.i("NotificationListener", "[snowdeer] Title:" + title);
    Log.i("NotificationListener", "[snowdeer] Text:" + text);
    Log.i("NotificationListener", "[snowdeer] Sub Text:" + subText);
  }

  @Override
  public void onNotificationRemoved(StatusBarNotification sbn) {
    Log.i("NotificationListener", "[snowdeer] onNotificationRemoved() - " + sbn.toString());
  }

}</pre>
<br>

참고로 이 서비스는 별도로 startService를 해주지 않아도, 사용자의 권한만 주어지면 자동으로 시작되기 때문에 startService를 구현할 필요는 없습니다.

사용자의 권한을 요청하는 코드는 다음과 같습니다.
<pre class="prettyprint">Intent intent = new Intent("android.settings.ACTION_NOTIFICATION_LISTENER_SETTINGS");
startActivity(intent);</pre>
<br>

하지만, 위 코드를 매번 호출하는 것은 사용자에게 상당히 번거로운 일이기 때문에, 다음과 같은 코드를 이용해서 기존에 사용자가 해당 App에 권한을 부여한 적이 있는지 확인하는 것이 좋습니다.

<br>

# MainActivity.java

<pre class="prettyprint">import android.content.Intent;
import android.support.v4.app.NotificationManagerCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import java.util.Set;

public class MainActivity extends AppCompatActivity {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    boolean isPermissionAllowed = isNotiPermissionAllowed();

    if(!isPermissionAllowed) {
      Intent intent = new Intent("android.settings.ACTION_NOTIFICATION_LISTENER_SETTINGS");
      startActivity(intent);
    }
  }

  private boolean isNotiPermissionAllowed() {
    Set&lt;String&gt; notiListenerSet = NotificationManagerCompat.getEnabledListenerPackages(this);
    String myPackageName = getPackageName();

    for(String packageName : notiListenerSet) {
      if(packageName == null) {
        continue;
      }
      if(packageName.equals(myPackageName)) {
        return true;
      }
    }

    return false;
  }
}</pre>
