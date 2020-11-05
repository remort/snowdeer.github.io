---
layout: post
title: Flutter for Desktop App (Mac OS)
category: Flutter

tag: [Flutter]
---

# Setup

<pre class="prettyprint">
flutter channel dev
flutter upgrade
flutter config --enable-macos-desktop
</pre>

<br>

# Device 확인

그 이후 `flutter devices` 명령어를 통해 현재 Mac PC가 발견되면 준비 끝입니다.

<pre class="prettyprint">
$ flutter devices

1 connected device:

macOS • macOS • darwin-x64 • Mac OS X 10.15.4 19E287
</pre>

<br>

# 실행

<pre class="prettyprint">
flutter create snowdeer_flutter
cd snowdeer_flutter

flutter run -d macos
</pre>