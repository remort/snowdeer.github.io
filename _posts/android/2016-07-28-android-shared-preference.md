---
layout: post
title: Shared Preference 사용하기
category: Android
tag: [Android]
---

보통 데이터를 저장할 때는 데이터베이스를 이용하는 경우가 많은데, 간단한 데이터만
저장하기 위해서 데이터베이스를 사용하는 것은 불필요한 작업일 경우가 많습니다.
이런 경우는 Shared Preference를 이용하면 간단하게 구현할 수 있습니다.

Shared Preference는 key-value 방식으로 데이터를 저장합니다.
그래서 원하는 데이터들을 쉽게 저장하고 불러올 수 있습니다.

다음은 Shared Preference를 사용하는 예제 코드입니다.

<br>

## SnowPreferenceManager.java

<pre class="prettyprint">package com.lnc.cuppadata.gui.renewal.main;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;


public class SnowPreferenceManager {

  public static final int VIEWMODE_DAY = 0;
  public static final int VIEWMODE_WEEK = 1;
  public static final int VIEWMODE_MONTH = 2;

  private static final String PREFERENCE_VIEW_MODE = "pref_view_mode";
  private static final String PREFERENCE_USER_ID = "pref_user_id";

  public static void setViewMode(Context context, int viewMode) {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
    SharedPreferences.Editor editor = prefs.edit();
    editor.putInt(PREFERENCE_VIEW_MODE, viewMode);
    editor.commit();
  }

  public static int getViewMode(Context context) {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
    return prefs.getInt(PREFERENCE_VIEW_MODE, VIEWMODE_DAY);
  }

  public static void setUserId(Context context, String userId) {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
    SharedPreferences.Editor editor = prefs.edit();
    editor.putString(PREFERENCE_USER_ID, userId);
    editor.commit();
  }

  public static String getUserId(Context context) {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
    return prefs.getString(PREFERENCE_USER_ID, "");
  }
}</pre>
