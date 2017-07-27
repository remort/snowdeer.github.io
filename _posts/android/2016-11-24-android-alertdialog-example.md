---
layout: post
title: AlertDialog 예제
category: Android
tag: [Android, UX]
---

안드로이드에서 간단한 AlertDialog를 띄우는 예제 코드입니다.

좀 더 자세한 정보와 예제 코드는 [여기를 참조](https://developer.android.com/reference/android/app/AlertDialog.html)하시면 됩니다.
<br>

# 예제 코드

<pre class="prettyprint">package com.lnc.datacafe;


import android.app.Activity;
import android.content.DialogInterface;
import android.os.Bundle;
import android.support.v7.app.AlertDialog;

public class MainActivity extends Activity {

  private AlertDialog mAlertDialog;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    setContentView(R.layout.activity_main);

    showAlertDialog();
  }

  private void showAlertDialog() {
    if(mAlertDialog != null) {
      mAlertDialog.dismiss();
      mAlertDialog = null;
    }

    mAlertDialog = new AlertDialog.Builder(this)
        .setTitle("여기에 Title을 입력하세요.")
        .setMessage("여기에 Message를 입력하세요")
        .setCancelable(true)
        .setPositiveButton("Ok", new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int id) {
            // TODO
          }
        })
        .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
          public void onClick(DialogInterface dialog, int id) {
            dialog.cancel();
          }
        }).create();

    mAlertDialog.show();
  }

}</pre>
