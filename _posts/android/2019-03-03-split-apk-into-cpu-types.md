---
layout: post
title: apk 빌드시 CPU 타입(arm64-v8a, armeabi-v7a, x86, x86_64 등)에 따라 분리해서 빌드하는 방법
category: Android
tag: [Android]
---

apk를 빌드할 때 모든 CPU 타입이 아닌, 원하는 타입에 대한 바이너리만 빌드할 수 있게 해주는 `build.gradle` 설정입니다.
전체 apk 용량도 줄어들기 떄문에 빌드 및 설치 시간도 단축이 됩니다.

<br>

## build.gradle

프로젝트가 아닌 모듈의 `build.gradle` 파일 설정입니다.

<pre class="prettyprint">
android {
    
    ...

    splits {
        abi {
            enable true
            reset()
            include "arm64-v8a", "armeabi-v7a"
            universalApk false
        }
    }
}
</pre>
