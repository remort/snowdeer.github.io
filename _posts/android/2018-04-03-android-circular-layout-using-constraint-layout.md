---
layout: post
title: Constraint Layout를 활용한 Circle Layout
category: Android
tag: [Android]
---
# Constraint Layout를 활용한 Circle Layout

Constraint Layout beta 3 부터 원형 레이아웃을 지원하기 시작했습니다. `gradle`에서 다음 종속성을 추가해주면 beta 3를 사용할 수 있습니다.

<br>

## gradle 설정

<pre class="prettyprint">
dependencies {
  implementation 'com.android.support.constraint:constraint-layout:1.1.0-beta3'
}
</pre>

<br>

## 사용 방법

![image](/assets/android/004.png)

위 그림을 참고해서 다음과 같은 레이아웃을 작성하면 원형 레이아웃을 만들 수 있습니다.

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.constraint.ConstraintLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  tools:context=".MainActivity"&gt;

  &lt;ImageView
    android:id="@+id/center_image"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:src="@mipmap/ic_launcher_round"
    app:layout_constraintBottom_toBottomOf="parent"
    app:layout_constraintLeft_toLeftOf="parent"
    app:layout_constraintRight_toRightOf="parent"
    app:layout_constraintTop_toTopOf="parent"/&gt;

  &lt;ImageView
    android:id="@+id/image_1"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:src="@mipmap/ic_launcher_round"
    app:layout_constraintCircle="@+id/center_image"
    app:layout_constraintCircleAngle="0"
    app:layout_constraintCircleRadius="140dp"/&gt;

  &lt;ImageView
    android:id="@+id/image_2"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:src="@mipmap/ic_launcher_round"
    app:layout_constraintCircle="@+id/center_image"
    app:layout_constraintCircleAngle="120"
    app:layout_constraintCircleRadius="140dp"/&gt;

  &lt;ImageView
    android:id="@+id/image_3"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:src="@mipmap/ic_launcher_round"
    app:layout_constraintCircle="@+id/center_image"
    app:layout_constraintCircleAngle="240"
    app:layout_constraintCircleRadius="140dp"/&gt;
&lt;/android.support.constraint.ConstraintLayout&gt;
</pre>