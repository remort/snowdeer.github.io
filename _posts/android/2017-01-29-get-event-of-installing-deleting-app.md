---
layout: post
title: Application의 설치 또는 삭제 이벤트 획득하기
category: Android
tag: [Android, Event]
---

안드로이드에서 어플리케이션(이하 App)을 설치하거나 삭제할 때는 그 이벤트가 Broadcast로
전달됩니다. 즉, BroadcastReceiver를 등록해놓은 각 App들의 설치/삭제 이벤트를 수신할 수 있습니다.

<br>

# AndroidManifest.xml

manifest.xml에 다음과 같이 BroadcastReceiver를 추가해줍니다.
<pre class="prettyprint">&lt;receiver android:name=".PackageEventReceiver"&gt;
    &lt;intent-filter&gt;
        &lt;action android:name="android.intent.action.PACKAGE_ADDED"/&gt;
        &lt;action android:name="android.intent.action.PACKAGE_REMOVED"/&gt;
        &lt;action android:name="android.intent.action.PACKAGE_REPLACED"/&gt;
        &lt;data android:scheme="package"/&gt;
    &lt;/intent-filter&gt;
&lt;/receiver&gt;</pre>
<br>

# PackageEventReceiver.java

<pre class="prettyprint">import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

import android.util.Log;

public class PackageEventReceiver extends BroadcastReceiver {

  @Override
  public void onReceive(Context context, Intent intent) {
    String packageName = intent.getData().getSchemeSpecificPart();
    String action = intent.getAction();

    if(action.equals(Intent.ACTION_PACKAGE_ADDED)) {
      Log.d("", "[snowdeer] Package ADDED : " + packageName);
    } else if(action.equals(Intent.ACTION_PACKAGE_REMOVED)) {
      Log.d("", "[snowdeer] Package REMOVED : " + packageName);
    }
  }
}</pre>
<br>

# Event Receiver 등록

<pre class="prettyprint">private PackageEventReceiver mPackageEventReceiver = new PackageEventReceiver();

private void registerPackageEventReceiver() {
  registerReceiver(mPackageEventReceiver, new IntentFilter(Intent.ACTION_PACKAGE_ADDED));
}</pre>

<br>

또는 다음과 같은 코드를 이용해서 등록하면 됩니다.
<pre class="prettyprint">private PackageEventReceiver mPackageEventReceiver = new PackageEventReceiver();

private void registerPackageEventReceiver() {
  IntentFilter intentFilter = new IntentFilter();
  intentFilter.addAction(Intent.ACTION_PACKAGE_ADDED);
  intentFilter.addAction(Intent.ACTION_PACKAGE_INSTALL);
  intentFilter.addDataScheme("package");
  registerReceiver(mPackageEventReceiver, intentFilter);
}</pre>
