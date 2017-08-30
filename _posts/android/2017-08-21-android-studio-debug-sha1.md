---
layout: post
title: Android Studio의 Debug에서의 SHA-1 값 알아내기
category: Android
tag: [Android, Android Studio, IDE]
---
키스토어(Keystore)의 SHA-1 값이 필요한 경우가 있습니다. [보통 Keystore를 직접 생성하고 
콘솔 창에서 SHA-1 값을 확인](/android/2017/01/23/android-how-to-use-google-api-key/)한다음 해당 Key를 이용해서 배포용 apk 파일을 만드는 경우가 많습니다.

하지만, 개발 도중에 Signed Key를 계속 적용하는 것은 번거롭기 때문에 여기서는 Android Sutdio에서
Debug 모드로 빌드할 때 사용하는 SHA-1 값을 확인하는 방법을 포스팅합니다.

<br>

# Android Studio에서 SHA-1 값 확인하기

![image -fullwidth](/assets/2017-08-21-android-studio-debug-sha1/01.png)

Android Studio 오른편에 보면 'Gradle'라는 버튼이 있습니다. Gradle 뷰안에서 SHA-1 값을 조회하기를 원하는 프로젝트를 선택한 다음 'Tasks → android → signingReport'를 더블 클릭하면 위 그림에서처럼 'Gradle Console'에 SHA-1 값이 출력됩니다.

~~~
Gradle 뷰 → 프로젝트 선택 → Tasks → android → signingReport
~~~

<br>

# Google 공식 가이드

Google에서는 [여기](https://developers.google.com/places/android-api/signup#debug-cert)에서 디버그 모드에서의 SHA-1 값을 획득하는 방법을 잘 설명하고 있습니다.

디버그 모드에서 사용하는 Keystore는 `debug.keystore` 파일이며, 이 파일의 위치는 

* Windows : C:\Users\your_user_name\.android\
* MacOS 및 Linux : ~/.android/

입니다. 해당 위치로 이동하여 다음 커맨드를 입력하면 SHA-1 값을 획득할 수 있습니다.

* Windows : keystore" -alias androiddebugkey -storepass android -keypass android
* MacOS 및 Linux : keytool -list -v -keystore ~/.android/debug.keystore -alias androiddebugkey -storepass android -keypass android