---
layout: post
title: Jacoco plugin 사용 방법
category: Android
tag: [Android]
---

[`jacoco`](https://www.eclemma.org/jacoco/)는 UnitTest 및 Coverage 레포트를 만들어주는 플러그인(plug-in)입니다.

`build.gradle`에 다음과 같이 작성하면 gradle 옵션에서 jacoco 레포트 생성을 선택할 수 있습니다.

<br>

## build.gradle (Module)

`excludes`는 레포트에서 제외할 파일이나 디렉토리, 패키지입니다. Kotlin이나 Java의 환경에 따라 `classDirectories`의 디렉토리 위치는 변경될 수 있습니다.

<pre class="prettyprint">
apply plugin: 'com.android.application'
apply plugin: 'kotlin-android'
apply plugin: 'kotlin-android-extensions'
apply plugin: 'jacoco'

jacoco {
    toolVersion = '0.8.4'
}

tasks.withType(Test) {
    jacoco.includeNoLocationClasses = true
}

task jacocoTestReport(type: JacocoReport, dependsOn: ['testDebugUnitTest', 'createDebugCoverageReport']) {

    reports {
        xml.enabled = true
        html.enabled = true
    }

    def excludes = [
            '**/R.class',
            '**/R$*.class',
            '**/BuildConfig.*',
            '**/Manifest*.*',
            'android/**/*.*',
            '**/treemanager/component/**',
            '**/treemanager/tree/*.*',
            '**/activity/**'
    ]

    classDirectories = fileTree(
            dir: "$buildDir/intermediates/javac/debug/compileDebugJavaWithJavac/classes",
            excludes: excludes
    ) + fileTree(
            dir: "$buildDir/tmp/kotlin-classes/debug",
            excludes: excludes
    )

    sourceDirectories = files([
            android.sourceSets.main.java.srcDirs,
            "src/main/kotlin"
    ])

    executionData = fileTree(dir: project.buildDir, includes: [
            'jacoco/testDebugUnitTest.exec', 'outputs/code_coverage/debugAndroidTest/connected/**/*.ec'
    ])
}

android {
    compileSdkVersion 28
    defaultConfig {
        applicationId "com.snowdeer.jacoco.example"
        minSdkVersion 26
        targetSdkVersion 28
        versionCode 1
        versionName "1.0"
        testInstrumentationRunner "android.support.test.runner.AndroidJUnitRunner"
        buildConfigField "long", "TIMESTAMP", System.currentTimeMillis() + "L"
    }
    buildTypes {
        debug {
            testCoverageEnabled true
        }

        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android-optimize.txt'), 'proguard-rules.pro'
        }
    }
    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }
    packagingOptions {
        pickFirst 'lib/arm64-v8a/*'
        pickFirst 'lib/armeabi-v7a/*'
    }
    buildToolsVersion '28.0.3'
}

</pre>

그런 다음 `gradle` 옵션에서 `jacocoTestReport` 항목을 선택하면 레포트가 만들어집니다.