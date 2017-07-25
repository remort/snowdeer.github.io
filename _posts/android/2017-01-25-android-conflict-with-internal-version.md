---
layout: post
title: Apache's Http Library 충돌 해결 방법
category: Android
tag: [Android, Apache]
---

[Apache's Http Library](https://hc.apache.org/httpcomponents-client-4.3.x/android-port.html)와
최신 버전의 Android SDK와 충돌이 발생하는 현상이 있습니다. 이럴 경우 다음과 같은 에러가 발생할 것입니다.

~~~
Warning:WARNING: Dependency org.apache.httpcomponents:httpclient:4.3.5 is ignored for debug as it may be conflicting with the internal version provided by Android.
~~~

그리고 빌드는 경고만 뜨고 잘 되더라도, 실행을 하려고 하면 에러가 뜨면서 실행이 안되는
현상이 생기기도 합니다. 이런 경우에는 app의 build.gradle 파일에 다음과 같은 코드를
삽입해주면 해결이 됩니다.

<br>
## build.gradle
<pre class="prettyprint">...
android {
   ...
    packagingOptions {
         exclude 'META-INF/NOTICE'
         exclude 'META-INF/LICENSE'
   }
}
</pre>
<br>

전체 코드는 다음과 같습니다.
<pre class="prettyprint">apply plugin: 'com.android.application'

android {
    compileSdkVersion 25
    buildToolsVersion "25.0.0"
    defaultConfig {
        applicationId "com.datacafe.nestcamapi"
        minSdkVersion 16
        targetSdkVersion 25
        versionCode 1
        versionName "1.0"
        testInstrumentationRunner "android.support.test.runner.AndroidJUnitRunner"
    }
    buildTypes {
        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android.txt'), 'proguard-rules.pro'
        }
    }
    packagingOptions {
        exclude 'META-INF/NOTICE'
        exclude 'META-INF/LICENSE'
    }
}

dependencies {
    compile fileTree(dir: 'libs', include: ['*.jar'])
    androidTestCompile('com.android.support.test.espresso:espresso-core:2.2.2', {
        exclude group: 'com.android.support', module: 'support-annotations'
    })
    compile 'com.android.support:appcompat-v7:25.0.1'
    testCompile 'junit:junit:4.12'
}
</pre>
