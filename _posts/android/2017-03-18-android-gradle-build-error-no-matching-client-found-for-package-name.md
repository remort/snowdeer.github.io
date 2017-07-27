---
layout: post
title: Gradle 빌드 오류 ‘No matching client found for package name’ 해결법
category: Android
tag: [Android, Android Studio]
---

Android Studio에서 Google Service를 사용하는 어플을 개발하다가 패키지 이름(Package Name)을
변경해야 할 경우가 생겼습니다. 그래서 리팩토링(Refactoring) 기능을 활용해서 패키지 이름을 바꾸었습니다.
혹시나 리팩토링 과정에서 생략되었을지도 모르는 `manifest.xml` 안의 내용과 `build.gradle` 안의
내용도 찾아서 수정하였습니다.

그런 다음, `빌드(Build)`를 수행했는데, 다음과 같은 오류가 발생했습니다.

~~~
Error:Execution failed for task ':app:processDebugGoogleServices'.
>; No matching client found for package name 'com.example.exampleapp'
~~~

<br>

# 해결법
해결법은 다음과 같았습니다.

먼저 Android Studio의 네비게이션 Bar의 보기 방식을 `Project`로 바꿔줍니다.
(기본은 `Android`로 되어 있습니다.)

![image -fullwidth](/assets/2017-03-18-android-gradle-build-error-no-matching-client-found-for-package-name/01.png)
그러면 'app' 폴더 아래 다음 이미지와 같이 'google-service.json' 파일이 보일 것입니다.
![image -fullwidth](/assets/2017-03-18-android-gradle-build-error-no-matching-client-found-for-package-name/02.png)
이 파일의 내부에서

<pre class="prettyprint">"client": [
  {
    "client_info": {
      "mobilesdk_app_id": "1:169851363875:android:33b601ddd12370e2",
      "android_client_info": {
        "package_name": "com.snowdeer.myexample"
      }
    },
    ...
  }
]</pre>
패키지 이름을 변경한 패키지 이름과 동일하게 맞추어 주면 됩니다. 이 파일 안에 패키지 이름이
등장하는 부분이 몇 군데 있는데, 전부 수정해주면 됩니다.
