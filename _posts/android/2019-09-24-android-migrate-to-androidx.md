---
layout: post
title: Migrate to AndroidX
category: Android
tag: [Android, Kotlin]
---

# androidx로 마이그레이션하기

2018년 Google IO에서 기존의 `android.support.*` 라이브러리들을 `AndroidX`로 교체하기로 발표했습니다.

기존에 만들어진 프로그램들은 Android Studio에서 `Migrate to AndroidX` 기능을 이용해서 마이그레이션할 수 있기는 한데
완벽하지 않기 때문에 수동으로 변경해줘야 하는 부분들이 존재합니다.

만약 `Migrate to AndroidX` 기능으로 마이그레이션이 정상적으로 되면, 프로젝트 내에 있는 
`gradle.properties` 파일에 다음과 같은 라인이 추가됩니다.

<pre class="prettyprint">
android.useAndroidX=true
android.enableJetifier=true
</pre>

<br>

## build.gralde (모듈)

또한, 각 모듈에 있는 `build.gradle` 은 다음과 같은 변화가 생깁니다.

<pre class="prettyprint">
androidTestImplementation 'com.android.support.test:runner:1.0.2'
androidTestImplementation 'com.android.support.test:rules:1.0.2'
androidTestImplementation 'com.android.support.test.espresso:espresso-core:3.0.2'

implementation 'com.android.support:appcompat-v7:28.0.0'
implementation 'com.android.support:design:28.0.0'
implementation 'com.android.support.constraint:constraint-layout:1.1.3'
</pre>

들이

<pre class="prettyprint">
androidTestImplementation 'androidx.test:runner:1.1.0'
androidTestImplementation 'androidx.test:rules:1.1.0'
androidTestImplementation 'androidx.test.espresso:espresso-core:3.1.0'

implementation 'androidx.appcompat:appcompat:1.0.0'
implementation 'com.google.android.material:material:1.0.0'
implementation 'androidx.constraintlayout:constraintlayout:1.1.3'
</pre>

## 컴포넌트 변경

그 외에도 많은 부분들이 변경됩니다.

변경 전 | 변경 후
-----: | :----
android.support.v4.widget.DrawerLayout | androidx.drawerlayout.widget.DrawerLayout
android.support.design.widget.NavigationView |com.google.android.material.navigation.NavigationView
android.support.design.widget.CoordinatorLayout |androidx.coordinatorlayout.widget.CoordinatorLayout
android.support.design.widget.AppBarLayout | com.google.android.material.appbar.AppBarLayout
android.support.v7.widget.Toolbar | androidx.appcompat.widget.Toolbar
android.support.v7.widget.CardView | androidx.cardview.widget.CardView

등과 같이 `support` 라이브러리의 대부분 컴포넌트가 `androidX`로 바뀌며, 위의 예시 외에도 `AppCompatActivity`나 `Fragment`, `AlertDialog` 등의 컴포넌트들도 전부 변경됩니다.