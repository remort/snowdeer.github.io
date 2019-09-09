---
layout: post
title: build.gradle 에서 변수 사용하기
category: Android
tag: [Android]
---

`gradle`에서 변수를 사용하는 방법입니다. 

예를 들어 특정 라이브러리의 버전을 변수로 지정할 수 있습니다.

## build.gradle (Project)

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.41'
    ext.snowlib_version = 'latest.integration'

    repositories {
        google()
        jcenter()
    }
    dependencies {
        classpath 'com.android.tools.build:gradle:3.4.2'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}
</pre>

<br>

## build.gradle (Module)

그리고 아래 예제와 같은 방법으로 위에서 선언한 `snowlib_version`이라는 변수를 사용할 수 있습니다.

<pre class="prettyprint">
dependencies {
    implementation fileTree(dir: 'libs', include: ['*.jar'])

    implementation project(':common_library')
    implementation 'com.google.code.gson:gson:2.8.5'

    implementation 'com.android.support:appcompat-v7:28.0.0'
    implementation 'com.android.support.constraint:constraint-layout:1.1.3'
    implementation 'com.android.support:design:28.0.0'
    testImplementation 'junit:junit:4.12'
    androidTestImplementation 'com.android.support.test:runner:1.0.2'
    androidTestImplementation 'com.android.support.test.espresso:espresso-core:3.0.2'

    implementation group: 'snowdeer.message.utils', name: 'snowdeer-message-util', version: "$snowlib_version", changing: true
    implementation group: 'snowdeer.actiontool.library', name: 'snowdeer-actiontool-library', version: "$snowlib_version", changing: true

    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"
}
</pre>