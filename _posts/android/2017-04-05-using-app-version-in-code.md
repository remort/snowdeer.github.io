---
layout: post
title: 현재 Application Version을 코드에서 사용하기
category: Android
tag: [Android]
---

안드로이드 App을 개발할 때 App Version을 소스 코드내에서 활용하고 싶을 때가 있습니다.

Eclipse의 경우에는 App Version이 `manifest.xml` 파일 내에 정의되어 있는데,
Android Studio에서는 `build.gradle` 내에 버전이 입력되어 있습니다.

<br>
## build.gradle
App Version은 build.gradle 내에 다음과 같이 설정됩니다.
<pre class="prettyprint">android {
    compileSdkVersion 25
    buildToolsVersion "25.0.0"
    defaultConfig {
        multiDexEnabled true
        applicationId "com.lnc.cuppadata"
        minSdkVersion 21
        targetSdkVersion 25
        versionCode 10
        versionName "0.10"
        testInstrumentationRunner "android.support.test.runner.AndroidJUnitRunner"
    }
    ...</pre>
<br.

그리고 안드로이드 소스내에서 App Version을 가져오는 코드는 다음과 같습니다.
## 소스 코드
<pre class="prettyprint">private String getAppVersion() {
  try {
    PackageInfo pInfo = getActivity().getPackageManager().getPackageInfo(
        getActivity().getPackageName(), 0);

    if(pInfo != null) {
      return pInfo.versionName;
    }
  } catch(Exception e) {
    e.printStackTrace();
  }

  return "";
}</pre>
