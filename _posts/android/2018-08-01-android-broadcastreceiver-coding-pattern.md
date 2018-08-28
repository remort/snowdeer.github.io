---
layout: post
title: BroadcastReceiver 코딩 템플릿
category: Android
tag: [Android]
---
# BroadcastReceiver 코딩 템플릿

다음은 `BroadcastReceiver` 프로그래밍 템플릿입니다. register/unregister를 BroadcastReceiver 외부에서 하는 방법도 있는데, 개인적으로는 해당 클래스 내부에 하는 것이 결합도는 낮아지고 응집력도 더 높아지며 코드도 깔끔해지는 것 같습니다. 

<pre class="prettyprint">
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

public class EventBroadcastReceiver extends BroadcastReceiver {

  static final String ACTION_EVENT_DEBUG_ON = "ACTION_EVENT_DEBUG_ON";
  static final String ACTION_EVENT_DEBUG_OFF = "ACTION_EVENT_DEBUG_OFF";

  final Context context;

  public EventBroadcastReceiver(Context ctx) {
    this.context = ctx;
  }

  public void init() {
    IntentFilter filter = new IntentFilter();
    filter.addAction(ACTION_EVENT_DEBUG_ON);
    filter.addAction(ACTION_EVENT_DEBUG_OFF);

    context.registerReceiver(this, filter);
  }

  public void fin() {
    context.unregisterReceiver(this);
  }

  @Override
  public void onReceive(Context context, Intent intent) {
    String action = intent.getAction();
    if (ACTION_EVENT_DEBUG_ON.equals(action)) {
      // TODO
    } else if (ACTION_EVENT_DEBUG_OFF.equals(action)) {
      // TODO
    }
  }
}
</pre>