---
layout: post
title: Foreground Service 용 Notification 꾸미기
category: Android
tag: [Android, Service]
---

안드로이드에서 서비스는 크게 백그라운드 서비스(Background Service)와
포그라운드 서비스(Foreground Service)로 나눌 수 있습니다.

<br>

## Background Service

우리가 흔히 말하는 서비스가 백그라운드 서비스 형태입니다. 시스템의 메모리가 부족할 경우
시스템이 해당 서비스를 강제로 종료시킬 수 있습니다. onStartCommand()의 파라메터를 이용하여,
서비스의 종료시 동작을 정의할 수 있습니다.
<ul>
 	<li>START_NOT_STICKY : 서비스가 종료되었을 때, 서비스 재 실행을 하지 않음</li>
 	<li>START_STICKY : 서비스가 종료되었을 때, 서비스를 재 실행 함. onStartCommand()를 호출하며 파라메터는 null 임</li>
 	<li>START_REDELIVER_INTENT : 서비스를 재 실행하며, 기존의 Intent 파라메터를 이용하여 onStartCommand()를 호출함</li>
</ul>
<br>

## Foreground Service

서비스의 우선 순위가 높아서, 시스템의 메모리가 부족하더라도 강제로 종료시키지 않습니다.
대신 상태바에 Notification이 표시됩니다. 과거에는 상태바에 Notification을 표시하지 않고도
Foreground Service로 실행할 수가 있었는데, 현재는 구글 정책으로 Foreground Service를 수행하기
위해서는 무조건 사용자에게 표시를 해야 하도록 변경되었습니다.

<br>

## Foreground Service 만들기

서비스는 기본적으로 Background Service 입니다. Foreground Service를 만들기 위해서는
서비스 내부에 다음과 같은 코드를 작성하면 됩니다.

<pre class="prettyprint">startForeground(1, new Notification());</pre>

이렇게 하면, 알아서 Notification을 표시해주며 서비스는 Foreground 형태로 동작하게 됩니다.

<br>

## Customize Notification 예제

이 때, Notification의 모습을 바꾸고 싶을 때는 어떻게 하면 될까요? 다음과 같은 코드를
이용해서 Notification의 모습을 바꿔줄 수 있습니다.

<br>

## notification_foreground.xml

<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="horizontal"&gt;

  &lt;ImageView
    android:layout_width="44dp"
    android:layout_height="44dp"
    android:layout_marginLeft="10dp"
    android:layout_gravity="center_vertical"
    android:src="@mipmap/ic_launcher" /&gt;

  &lt;LinearLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:layout_margin="10dp"
    android:orientation="vertical"&gt;

    &lt;TextView
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:layout_gravity="center_vertical"
      android:text="@string/notification_title"
      android:textColor="@color/black"
      android:textSize="18sp" /&gt;

    &lt;TextView
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:layout_gravity="center_vertical"
      android:text="@string/notification_message"
      android:textColor="@color/darkgray"
      android:textSize="14sp" /&gt;
  &lt;/LinearLayout&gt;
&lt;/LinearLayout&gt;</pre>

<br>

## SampleService.java

<pre class="prettyprint">RemoteViews remoteViews = new RemoteViews(getPackageName(),
    R.layout.notification_foreground);
Notification.Builder mBuilder = new Notification.Builder(this)
    .setSmallIcon(R.mipmap.ic_launcher).setContent(remoteViews);

startForeground(1, mBuilder.build());</pre>
<br>

여기에 Notification을 눌렀을 때 동작하는 [PendingIntent](https://developer.android.com/reference/android/app/PendingIntent.html)를
연결하고 싶으면 다음과 같이 작성하면 됩니다.

<pre class="prettyprint">Intent notificationIntent = new Intent(this, MainActivity.class);
PendingIntent pendingIntent = PendingIntent.getActivity(this, 0, notificationIntent, 0);
RemoteViews remoteViews = new RemoteViews(getPackageName(),
    R.layout.notification_foreground);
Notification.Builder mBuilder = new Notification.Builder(this)
    .setSmallIcon(R.mipmap.ic_launcher)
    .setContent(remoteViews)
    .setContentIntent(pendingIntent);
startForeground(1, mBuilder.build());</pre>
