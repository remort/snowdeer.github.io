---
layout: post
title: Android Studio Gradle 에서 빌드시 Signed Key 설정하기
category: Android
tag: [android, build, signed-key]
---

Android Studio에서 빌드할 때 Signed Key를 설정하는 방법을 포스팅해봅니다.

물론, Release로 빌드시 메뉴에서 'Generate Signed APK'를 선택하여
빌드하는 방법도 있습니다. 하지만, 최종 배포뿐만 아니라 항상 빌드할 때마다 Signed Key를 
설정해야 하는 경우도 있을 수 있습니다. 그리고 Relase가 아닌 Debug로 빌드할 때도 
Signed Key를 설정해야 하는 경우도 발생할 수 있습니다.

Eclipse에서는 비교적 메뉴에서 쉽게 Signed Key를 사용하도록 설정할 수 있었지만,
Android Studio에서는 gradle에 해당 내용을 세팅해야 합니다.

app의 build.gradle 파일을 엽니다. 
그리고 아래와 같이

<pre class="prettyprint" style="font-size:0.7em;">
signingConfigs{
        releaseWithSignedKey {
            storeFile file("SampleKeyStore.jks")
            storePassword "password1"
            keyAlias "sample"
            keyPassword "password2"
        }
    }
</pre>

항목을 만들어줍니다.
그리고, buildType에 아래와 같이

<pre class="prettyprint" style="font-size:0.7em;">
buildTypes {
        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android.txt'), 'proguard-rules.pro'
            signingConfig signingConfigs.releaseWithSignedKey
        }
    }
</pre>

와 같이 설정해주면 됩니다. Debug 모드에서도 Signed Key를 참조하게 하려면?

<pre class="prettyprint" style="font-size:0.7em;">
buildTypes {
        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android.txt'), 'proguard-rules.pro'
            signingConfig signingConfigs.releaseWithSignedKey
        }
        debug{
            signingConfig signingConfigs.releaseWithSignedKey
        }
    }
</pre>

와 같이 추가로 작성을 해주면 됩니다.

그런데, 이렇게 하면 build.gradle에 KeyStore의 민감한 정보들이 너무 노출이 되고 관리도 번거로워지기 때문에
각 변수 값들을 변수 처리해주는 것이 깔끔합니다.

이 때는 Project의 gradle.properties에

<pre class="prettyprint" style="font-size:0.7em;">
SIGNED_STORE_FILE=SampleKeyStore.jks
SIGNED_STORE_PASSWORD=password1
SIGNED_KEY_ALIAS=sample
SIGNED_KEY_PASSWORD=password2
</pre>

와 같이 작성하고, build.gradle에는

<pre class="prettyprint" style="font-size:0.7em;">
signingConfigs{
        releaseWithSignedKey {
            storeFile file(SIGNED_STORE_FILE)
            storePassword SIGNED_STORE_PASSWORD
            keyAlias SIGNED_KEY_ALIAS
            keyPassword SIGNED_KEY_PASSWORD
        }
    }
</pre>

와 같이 변경해줍니다.


그리고 Run을 해보면, Signed Key가 정상적으로 적용된 것을 알 수 있습니다.

참고로, 전체 build.gradle 파일 첨부합니다.

<pre class="prettyprint" style="font-size:0.7em;">
apply plugin: 'com.android.application'

android {
    compileSdkVersion 25
    buildToolsVersion "25.0.0"
    defaultConfig {
        applicationId "com.datacafe.googleawarenessapi"
        minSdkVersion 15
        targetSdkVersion 25
        versionCode 1
        versionName "1.0"
        testInstrumentationRunner "android.support.test.runner.AndroidJUnitRunner"
    }

    signingConfigs{
        releaseWithSignedKey {
            storeFile file(SIGNED_STORE_FILE)
            storePassword SIGNED_STORE_PASSWORD
            keyAlias SIGNED_KEY_ALIAS
            keyPassword SIGNED_KEY_PASSWORD
        }
    }

    buildTypes {
        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android.txt'), 'proguard-rules.pro'
            signingConfig signingConfigs.releaseWithSignedKey
        }
        debug{
            signingConfig signingConfigs.releaseWithSignedKey
        }
    }
}

dependencies {
    compile fileTree(dir: 'libs', include: ['*.jar'])
    androidTestCompile('com.android.support.test.espresso:espresso-core:2.2.2', {
        exclude group: 'com.android.support', module: 'support-annotations'
    })
    compile 'com.android.support:appcompat-v7:25.0.1'


    compile 'com.google.android.gms:play-services:10.0.1'

    testCompile 'junit:junit:4.12'
}
</pre>
