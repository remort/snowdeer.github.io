---
layout: post
title: Android Studio에서 모듈(Module)간 종속성 지정하기
category: Android
tag: [Android, UX]
---

Android Studio와 Eclipse에서의 프로젝트(Project)의 개념이 조금 다릅니다.

Eclipse에서는 하나의 실행 파일(*.apk), 라이브러리(*.jar) 등을 생성하는 단위를 프로젝트라고 불렀으며, Android Studio에서는 이를 모듈(Module)이라고 부르고 있습니다.

그리고 이러한 모듈들의 모음을 Android Studio에서는 프로젝트라는 단위로 명칭하고 있습니다.

<br>

# 모듈 생성 방법

Android Studio에서 모듈은 다음과 같이 `New Module` 메뉴를 이용해서 생성할 수 있습니다.

![Image](/assets/2017-08-07-android-studio-dependencies-among-modules/01.png)

<br>

이 때, 추가하려는 모듈이 별도의 실행 파일인지 라이브러리인지 기타 모듈인지 선택할 수 있습니다.

![Image](/assets/2017-08-07-android-studio-dependencies-among-modules/02.png)

<br>

# 모듈간 종속성(Dependency) 설정

위와 같은 방법으로 모듈을 추가했다면, 이제 모듈간 종속성을 설정할 수 있습니다. 만약 라이브러리로 추가했다면(Eclipse에서는 라이브러리가 *.jar 파일이었고, Android Sutdio에서는 *.aar 입니다.), 그 라이브러리를 사용하려는 프로젝트의 `build.gradle`에 다음과 같은 코드를 추가하면 됩니다.

<pre class="prettyprint">
dependencies {
    ...
    compile project(':snowsdk');
}
</pre>