---
layout: post
title: Flutter 기존 프로젝트에 MacOS 실행 환경 추가
category: Flutter

tag: [Flutter]
---

# Flutter 기존 프로젝트에 MacOS 실행 환경 추가

기존에 만들어진 Flutter 프로젝트에 MacOS 실행 환경을 추가하는 방법입니다. 
리눅스 PC에서 생성한 Flutter 프로젝트를 MacOS에서 실행하면 처음에 다음과 같은 메시지가 발생합니다.

<pre class="prettyprint">
flutter run -d macos
Launching lib/main.dart on macOS in debug mode...
Exception: No macOS desktop project configured. See
https://flutter.dev/desktop#add-desktop-support-to-an-existing-flutter-project to learn about adding
macOS support to a project.
</pre>

<br>

## 해결법

`flutter create .` 명령어를 실행하면 됩니다.

<pre class="prettyprint">
$ flutter create .

Recreating project ....
  snowdeer_flutter_sample.iml (created)
  macos/Runner.xcworkspace/contents.xcworkspacedata (created)
  macos/Runner.xcworkspace/xcshareddata/IDEWorkspaceChecks.plist (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_16.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_1024.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_256.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_64.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_512.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_128.png (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/Contents.json (created)
  macos/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_32.png (created)
  macos/Runner/DebugProfile.entitlements (created)
  macos/Runner/Base.lproj/MainMenu.xib (created)
  macos/Runner/MainFlutterWindow.swift (created)
  macos/Runner/Configs/Debug.xcconfig (created)
  macos/Runner/Configs/Release.xcconfig (created)
  macos/Runner/Configs/Warnings.xcconfig (created)
  macos/Runner/Configs/AppInfo.xcconfig (created)
  macos/Runner/AppDelegate.swift (created)
  macos/Runner/Info.plist (created)
  macos/Runner/Release.entitlements (created)
  macos/Runner.xcodeproj/project.xcworkspace/xcshareddata/IDEWorkspaceChecks.plist (created)
  macos/Runner.xcodeproj/project.pbxproj (created)
  macos/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme (created)
  macos/Flutter/Flutter-Debug.xcconfig (created)
  macos/Flutter/Flutter-Release.xcconfig (created)
  macos/.gitignore (created)
  android/snowdeer_flutter_sample_android.iml (created)
  .idea/runConfigurations/main_dart.xml (created)
  .idea/libraries/KotlinJavaRuntime.xml (created)
Wrote 33 files.

All done!
</pre>