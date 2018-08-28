---
layout: post
title: Oreo 버전 이후 Foreground Service 사용 방법
category: Android
tag: [Android]
---
#  Oreo 버전 이후 Foreground Service 사용 방법

안드로이드 Oreo 버전 이후부터는 Foreground Service를 사용하는 방법이 약간 변경되었습니다. Foreground Service를 만들기 위한 조건은 다음과 같습니다.

* 상태바에Notification을 등록해서 사용자가 인지할 수 있게 해야 한다.
* Notification의 id는 0이 아니어야 한다.

예제 코드는 다음과 같습니다. Notification을 클릭하면 MainActivity로 이동하도록 `PendingIntent`를 등록했으며, 사용자 정의 Notification을 보여주도록 했습니다.

<br>

## SnowDeerService.java

<pre class="prettyprint">
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.IBinder;
import android.support.annotation.Nullable;
import android.support.v4.app.NotificationCompat;
import android.support.v4.app.NotificationCompat.Builder;
import android.widget.RemoteViews;

public class SnowDeerService extends Service {

  @Nullable
  @Override
  public IBinder onBind(Intent intent) {
    return null;
  }

  @Override
  public void onCreate() {
    super.onCreate();

    startForegroundService();
  }

  void startForegroundService() {
    Intent notificationIntent = new Intent(this, MainActivity.class);
    PendingIntent pendingIntent = PendingIntent.getActivity(this, 0, notificationIntent, 0);

    RemoteViews remoteViews = new RemoteViews(getPackageName(), R.layout.notification_service);

    NotificationCompat.Builder builder;
    if (Build.VERSION.SDK_INT >= 26) {
      String CHANNEL_ID = "snwodeer_service_channel";
      NotificationChannel channel = new NotificationChannel(CHANNEL_ID,
          "SnowDeer Service Channel",
          NotificationManager.IMPORTANCE_DEFAULT);

      ((NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE))
          .createNotificationChannel(channel);

      builder = new Builder(this, CHANNEL_ID);
    } else {
      builder = new Builder(this);
    }
    builder.setSmallIcon(R.mipmap.ic_launcher)
        .setContent(remoteViews)
        .setContentIntent(pendingIntent);

    startForeground(1, builder.build());
  }
}
</pre>

Oreo 버전 부터는 Notification에 채널 ID를 지정해줘야 합니다. 그리고 Notification을 등록하기 위해서는 아이콘, 제목, 내용이 전부 있어야 가능합니다.

서비스를 호출하는 방법도 변경이 있습니다. 기존의 `startService()` 대신 `startForegroundService()` 라는 메소드를 이용해서 서비스를 시작해야 합니다.

<br>

## MainFragment.java

저는 Fragment 내부의 버튼을 이용해서 서비스 시작을 했지만, Actvity의 경우에도 동일합니다.

<pre class="prettyprint">
@Override
  public View onCreateView(LayoutInflater inflater, ViewGroup container,
      Bundle savedInstanceState) {
    // Inflate the layout for this fragment
    View view = inflater.inflate(R.layout.fragment_main, container, false);

    view.findViewById(R.id.start_service_button).setOnClickListener(view1 -> {
      Intent intent = new Intent(getContext(), SnowDeerService.class);
      if (Build.VERSION.SDK_INT >= 26) {
        getContext().startForegroundService(intent);
      }
      else {
        getContext().startService(intent);
      }
    });

    view.findViewById(R.id.stop_service_button).setOnClickListener(view12 -> {
      Intent intent = new Intent(getContext(), SnowDeerService.class);
      getContext().stopService(intent);
    });

    return view;
  }
</pre>