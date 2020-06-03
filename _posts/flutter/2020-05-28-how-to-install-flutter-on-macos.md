---
layout: post
title: Flutter 설치 방법(MacOS)
category: Flutter

tag: [Flutter]
---

# Flutter 설치 방법

Flutter를 Mac OS에 설치 하는 방법입니다.

[여기](https://flutter.dev/docs/get-started/install)에서 Flutter SDK를 다운로드할 수 있으며, 그 중 Mac OS 버전은 [여기](https://flutter.dev/docs/get-started/install/macos)에서 받을 수 있습니다.

설치 후 다음 명령어로 Flutter를 설치합니다.

<pre class="prettyprint">
mkdir ~/Development
cd ~/Development
unzip ~/Downloads/flutter_macos_1.17.2-stable.zip
</pre>

그리고 `.zshrc` 파일에 아래 부분을 추가합니다.

<pre class="prettyprint">
# for Flutter
export PATH=$PATH:~/Development/flutter/bin
</pre>

<br>

# Pre-download Development Binaries

<pre class="prettyprint">
flutter precache
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

XCode 설치가 불완전하면 

<pre class="prettyprint">
sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer
sudo xcodebuild -runFirstLaunch

sudo gem install cocoapods
</pre>

를 실행합니다.

`Android Studio`나 `IntelliJ`, `VSCode` 같은 경우는 `Flutter` 플러그인을 설치해줍니다.

<br>

# Flutter 프로젝트 생성하기

<pre class="prettyprint">
flutter create snowdeer_flutter
</pre>