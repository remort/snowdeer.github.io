---
layout: post
title: ProgressDialog 예제
category: Android
tag: [Android, UX]
---

ProgressBar는 로딩 등과 같이 어떤 작업이 수행되고 있다는 것을 사용자에게 알려주기 위한
UX 컴포넌트입니다. 원형, 선형 등과 같이 다양한 형태의 ProgressBar가 존재합니다.
그리고 이러한 ProgressBar를 Dialog 형태인 ProgressDialog로도 보여줄 수 있습니다.

좀 더 자세한 정보는 [여기를 참조](https://developer.android.com/reference/android/app/ProgressDialog.html)하시면
됩니다.

<br>

# 예제 코드

<pre class="prettyprint">private ProgressDialog mProgressDialog;

private void showProgressDialog(String message) {
  closeProgressDialog();
  mProgressDialog = new ProgressDialog(this);
  mProgressDialog.setTitle("");
  mProgressDialog.setMessage(message);
  mProgressDialog.setCancelable(true);
  mProgressDialog.setIndeterminate(true);
  mProgressDialog.show();
}

private void closeProgressDialog() {
  if((mProgressDialog != null) &amp;&amp; (mProgressDialog.isShowing())) {
    mProgressDialog.dismiss();
  }
  mProgressDialog = null;
}</pre>
