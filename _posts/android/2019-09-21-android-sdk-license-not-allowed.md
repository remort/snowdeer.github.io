---
layout: post
title: Warning: License for package Android SDK Build-Tools 28.0.3 not accepted.
category: Android
tag: [Android, Kotlin]
---

MacOS 기준으로 설명합니다. (해결 방법은 Windows나 Ubuntu에서도 비슷합니다.)

# License for package Android SDK not accepted.

만약 안드로이드 스튜디오를 설치하고 나서 다음과 같은 오류가 발생했을 경우에는

<pre class="prettyprint">
Warning: License for package Android SDK Build-Tools 28.0.3 not accepted.
</pre>

터미널을 열고 다음 명령어를 입력합니다.

<pre class="prettyprint">
cd ~/Library/Android/sdk/tools/bin
./sdkmanager --licenses
</pre>

Windows에서는 다음 명령어를 입력하면 됩니다.

<pre class="prettyprint">
cd "%ANDROID_HOME%"/tools/bin
sdkmanager --licenses
</pre>

위 명령어를 입력하면 Lincese 수락을 할 수 있는 창이 뜹니다.
또는 `Java Runtime`을 설치하라는 메시지가 뜨기도 합니다. 이 경우는 JDK를 [다운로드](https://www.oracle.com/technetwork/java/javase/downloads/index.html?ssSourceSiteId=otnjp)해서 설치하면 해결됩니다.

<br>

## Failed to install the following Android SDK packages as some licences have not been accepted.

만약

<pre class="prettyprint">
ERROR: Failed to install the following Android SDK packages as some licences have not been accepted.
   build-tools;28.0.3 Android SDK Build-Tools 28.0.3
   platforms;android-28 Android SDK Platform 28
To build this project, accept the SDK license agreements and install the missing components using the Android Studio SDK Manager.
Alternatively, to transfer the license agreements from one workstation to another, see http://d.android.com/r/studio-ui/export-licenses.html

Using Android SDK: /Users/snowdeer/Library/Android/sdk
</pre>

와 같은 오류 메시지가 나타날 경우는 Android SDK 일부가 설치가 덜 된 경우이기 때문에 `SDK Manager`를 이용해 부족한 컴포넌트를 다시 설치해주면
문제가 해결됩니다.