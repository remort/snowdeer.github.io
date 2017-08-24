---
layout: post
title: Intent Service 예제
category: Android
tag: [Android]
---
오래 걸리는 작업은 인텐트 서비스(Intent Service)를 이용해서 실행할 수 있습니다.
예를 들어 특정 Broadcast 메세지를 수신한 Broadcast Receiver에서 시간이 오래걸리면
'ANR(Application Not Responding)' 에러가 발생할 수도 있습니다.

이런 경우는 인텐트 서비스를 이용하면 수월하게 해결 할 수 있습니다.

예제는 다음과 같습니다.

<br>

# SnowBroadcastReceiver.java

<pre class="prettyprint">
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

public class SnowBroadcastReceiver extends BroadcastReceiver {

  @Override
  public void onReceive(Context context, Intent intent) {
    Intent newIntent = new Intent(context, SnowIntentService.class);
    context.startService(newIntent);
  }
}
</pre>

<br>

# SnowIntentService.java

<pre class="prettyprint">
import android.app.IntentService;
import android.content.Intent;
import android.support.annotation.Nullable;
import android.util.Log;

public class SnowIntentService extends IntentService {

  public SnowIntentService(String name) {
    super(name);
  }

  @Override
  protected void onHandleIntent(@Nullable Intent intent) {
    long sum = 0;
    for (int i = 0; i < 100000; i++) {
      sum += i;
    }

    try {
      Thread.sleep(60 * 1000);
      Log.i("", "Sum : " + sum);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
</pre>