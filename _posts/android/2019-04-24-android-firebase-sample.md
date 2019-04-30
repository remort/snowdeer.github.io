---
layout: post
title: Firebase 프로젝트 세팅하기
category: Android
tag: [Android]
---

# Firebase 계정 설정

Firebase Application을 작성하기 위해서는 먼저 Firebase 계정이 필요합니다.

먼저 [Firebase Console](https://console.firebase.google.com/)에 로그인합니다.
그리고 신규 프로젝트 추가를 실행합니다.


![image](/assets/android-firebase/001.png)

그리고 화면에서 `Android` 버튼을 눌러서 안드로이드 앱을 추가합니다.

<br>

![image](/assets/android-firebase/002.png)

앱 등록 화면에서는 패키지 이름을 넣도록 합시다. 

<br>

![image](/assets/android-firebase/003.png)

구성 파일 다운로드 화면이 나오는데, `google-services.json` 파일을 다운로드 해서 모듈의 루트 디렉토리에 복사해줍니다.

<br>

![image](/assets/android-firebase/004.png)

그리고 프로젝트의 `build.gradle` 파일과 모듈의 `build.gradle` 파일에 다음과 같은 플러그인을 설정해줍니다.

### build.gradle (프로젝트)

<pre class="prettyprint">
buildscript {
  dependencies {
    // Add this line
    classpath 'com.google.gms:google-services:4.0.1'
  }
}
</pre>

### build.gradle (모듈)

### build.gradle (프로젝트)

<pre class="prettyprint">
dependencies {
  // Add this line
  implementation 'com.google.firebase:firebase-core:16.0.1'
}

...

// Add to the bottom of the file
apply plugin: 'com.google.gms.google-services'
</pre>

<br>

![image](/assets/android-firebase/005.png)

위 단계까지 설정하고 나서 단말에서 어플리케이션을 실행해주면 Firebase 서버에서 어플리케이션 설정이 정상적으로 되었는지 체크합니다.

위 과정을 거치면 본격적으로 Firebase 어플리케이션을 작성할 수 있습니다.