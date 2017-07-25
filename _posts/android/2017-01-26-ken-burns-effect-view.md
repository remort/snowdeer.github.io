---
layout: post
title: Ken Burns Effect View
category: Android
tag: [Android, UX, Open Source]
---

[Ken Burns Effect](https://en.wikipedia.org/wiki/Ken_Burns_effect)는
패닝(Panning)과 줌(Zooming) 기반으로 사진을 화려하게 보여줄 수 있는 효과(Effect)로
사진으로부터 동영상 등을 뽑아낼 때 많이 사용하고 있습니다.

Ken Burns Effect의 모습은 다음과 같습니다.

![KenBurns]({{ site.baseurl }}/assets/2017-01-26-ken-burns-effect-view/anim.gif)

이미 아주 많은 사람들이 사용하고 있는 오픈소스가 있으니 그걸 활용하도록 하겠습니다.
[여기에서 소스 코드를 확인](https://github.com/flavioarfaria/KenBurnsView)할 수 있으며,
Android Studio에서는 간단히 gradle에 다음 라인만 추가하면 KenBurnsView를 사용할 수 있습니다.

<br>
## Dependency 설정
<pre class="prettyprint">dependencies {
    compile 'com.flaviofaria:kenburnsview:1.0.7'
}
</pre>
<br>
## Layout 코드
실제로 사용할 때는 다음과 같이 XML에 KenburnsView를 배치하기만 하면 됩니다.
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;RelativeLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:id="@+id/activity_main"
  android:layout_width="match_parent"
  android:layout_height="match_parent"&gt;

  &lt;com.flaviofaria.kenburnsview.KenBurnsView
    android:id="@+id/image"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:src="@drawable/image" /&gt;

&lt;/RelativeLayout&gt;</pre>
<br>
특별히 Java 코드를 따로 추가하지 않더라도 훌륭한 Ken Burns 효과를 보여주며, Java 코드를 통해 좀 더 다양하고 정교한 동작을 설정할 수 있습니다.
