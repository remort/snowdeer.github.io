---
layout: post
title: 메소드 개수가 65536개 이상일 때 빌드가 안되는 현상
category: Android
tag: [Android, Android Studio]
---

안드로이드 어플리케이션을 개발할 때 메소드 개수가 65536개 이상이 되어 빌드가 되지 않는
현상이 있습니다. 빌드시 발생하는 오류 메세지는 다음과 같습니다.

<br>

# 오류메세지

~~~
Execution failed for task ':app:transformClassesWithDexForDebug'.
>; com.android.build.api.transform.TransformException:
com.android.ide.common.process.ProcessException:
java.util.concurrent.ExecutionException:
com.android.dex.DexIndexOverflowException:
method ID not in [0, 0xffff]: 65536
~~~
DexIndex가 오버플로(Overflow)되었으며, 메소드 ID(method ID)를 인덱스내에서 발견하지 못했다는 메세지입니다.

<br>

# 해결법

이 경우는 몇 가지 해결책이 있습니다. 보통, 메소드 개수가 65536개를 넘는 경우가 흔하지는 않습니다.
이를 넘긴 경우는 주로 외부 오픈 소스들을 남발한 경우가 대부분일 것입니다.
즉, 이런 경우는 다음과 같은 방법으로 해결할 수 있습니다.

<ul>
 	<li>참조하는 외부 라이브러리 개수를 줄이고, 필요한 건 직접 구현해서 사용하는 방법</li>
 	<li>Proguard 등을 이용하여 사용하지 않는 메소드를 삭제하는 방법</li>
 	<li>Multidex 기능을 이용하는 방법</li>
</ul>

가장 좋은 건 리팩토링을 통해 구조 개선을 하고, 꼭 필요한 라이브러리만 참조하는 방법입니다.
하지만, 이 방법은 시간과 노력이 많이 드는 방법이라 일단 Multidex 기능을 이용하는 방법을
알아보도록 하겠습니다.

참고로, 이 문제에 대한 [구글의 공식 설명](https://developer.android.com/studio/build/multidex.html?hl=ko)이
존재합니다.

<br>

# Multidex 활용한 해결 방법

## build.gradle 수정

defaultConfig에 다음과 같이 'multiDexEnabled true' 항목을 추가합니다.
<pre class="prettyprint">defaultConfig {
    ...
    multiDexEnabled true
    ...
}</pre>
그리고 dependencies에도 다음 항목을 추가해줍니다.
<pre class="prettyprint">dependencies {
    ...
    compile 'com.android.support:multidex:1.0.0'
    ...
}</pre>

<br>

## Application 상속

그리고 메인 Application을 다음과 같이 'android.support.multidex.MultiDexApplication'을 상속받도록 수정합니다.
<pre class="prettyprint">import android.support.multidex.MultiDexApplication;

public class MyApplication extends MultiDexApplication {
  ...
}</pre>
만약 Application을 Java 클래스로 구현해서 사용하고 있지 않다면, manifest.xml에서 application을 다음과 같이 수정하면 됩니다.

<br>

## Manifest.xml
<pre class="prettyprint">&lt;application
  android:name="android.support.multidex.MultiDexApplication"
  ...&gt;</pre>
