---
layout: post
title: 의존 라이브러리의 버전을 항상 최신으로 사용하도록 하는 gradle 옵션
category: Android
tag: [Android]
---

기본적으로 의존 라이브러리를 사용하기 위해서는 `build.gradle`안에 다음과 같이 선언합니다.

<pre class="prettyprint">
dependencies {
    implementation group: 'com.android.support', name: 'appcompat-v7', version: '28.0.0'
}
</pre>

위와 같이 사용하거나 짧게 줄여서 `group:name:version` 형태로 사용할 수도 있습니다.

<pre class="prettyprint">
dependencies {
    implementation 'com.android.support:appcompat-v7:28.0.0'
}
</pre>

만약 항상 최신 버전의 라이브러리를 사용하고 싶을 때는 `version` 값에 `latest.integration`을 넣으면 됩니다.

