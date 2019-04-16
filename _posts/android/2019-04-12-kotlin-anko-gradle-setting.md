---
layout: post
title: Kotlin Anko gradle 세팅
category: Android
tag: [Android, Kotlin]
---

## build.gradle(프로젝트)

<pre class="prettyptint">
buildscript {
    ext.kotlin_version = '1.3.21'
    ext.anko_version='0.10.8'
    }
}
</pre>

<br>

## build.gradle(모듈)

<pre class="prettyptint">
dependencies {
    implementation fileTree(dir: 'libs', include: ['*.jar'])
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"
    implementation "org.jetbrains.anko:anko:$anko_version"
}
</pre>