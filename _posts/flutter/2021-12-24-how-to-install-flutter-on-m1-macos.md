---
layout: post
title: Flutter 설치 방법(M1 MacOS)
category: Flutter

tag: [Flutter]
---

# Flutter 설치 방법

Flutter를 Mac OS에 설치 하는 방법입니다. 이전에도 한 번 포스팅을 했지만, Flutter 버전이 올라가면서 설치 방법에 
아주 약간의 변경이 있었네요. 큰 차이는 없습니다.

[여기](https://flutter.dev/docs/get-started/install)에서 Flutter SDK를 다운로드할 수 있으며, 그 중 MacOS 버전은 [여기](https://flutter.dev/docs/get-started/install/macos)에서 받을 수 있습니다.

설치 후 다음 명령어로 Flutter를 설치합니다.

<pre class="prettyprint">
mkdir ~/Development
cd ~/Development
unzip ~/Downloads/flutter_macos_2.8.1-stable.zip
</pre>

그리고 `.zshrc` 파일에 아래 부분을 추가합니다.

<pre class="prettyprint">
# for Flutter
export PATH=$PATH:~/Development/flutter/bin
</pre>

<br>

# Flutter doctor

설치 후 `flutter doctor` 명령어를 이용해서 현재 시스템에 부족한 부분이 어떤 것이 있는 지 점검할 수 있습니다.

<pre class="prettyprint">
flutter doctor
</pre>

<br>

## 점검 사항

안드로이드 라이센스가 수락되어 있지 않으면 

<pre class="prettyprint">
flutter doctor --android-licenses
</pre>

를 해줍니다.

XCode 라이센스 역시 아래의 명령어로 해결할 수 있습니다.

<pre class="prettyprint">
sudo xcodebuild -license
</pre>

를 실행합니다.

iOS 개발을 할거면 `CocoaPods`도 설치해줍니다. 필수는 아니지만, [Flutter Plugin](https://docs.flutter.dev/development/packages-and-plugins/developing-packages#types) 개발을 위해서도 설치가 필요합니다.

<pre class="prettyprint">
sudo gem install cocoapods
</pre>

<br>

## Enable desktop support

MacOS용 어플리케이션을 개발하기 위해서는 아래 설정을 꼭 해줘야 합니다.

<pre class="prettyprint">
flutter config --enable-macos-desktop
</pre>

## IntelliJ 설치 및 flutter 플러그인 설치

이제 [IntelliJ를 설치](https://www.jetbrains.com/idea/)하고 IntelliJ 내의 flutter 플러그인을 설치하면
개발 환경 설정 완료입니다.