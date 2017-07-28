---
layout: post
title: 간단한 애니메이션 예제
category: Android
tag: [Android, UX]
---

안드로이드에서 간단한 애니메이션(Animation)을 구현하는 방법을 알아보도록 하겠습니다.

<br>

# 투명도 조절

다음 XML 파일을 `res/anim` 폴더 아래에 만들어줍니다. 만약, `anim` 폴더가 존재하지 않는 경우는 직접 만들어주면 됩니다.

## alpha.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;alpha xmlns:android="http://schemas.android.com/apk/res/android"
  android:duration="3000"
  android:fromAlpha="0.0"
  android:interpolator="@android:anim/accelerate_interpolator"
  android:toAlpha="1.0" /&gt;
</pre>

투명도 값을 의미하는 `alpha` 값은 `0.0`이 완전 투명, `1.0`이 완전 불투명을 의미합니다. 그리고 `duration`의 단위는 `msec(1/1000 초)`입니다.

<br>

## MainActivity.java

<pre class="prettyprint">
public class snowSample extends Activity {
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.snow_intro);

    ImageView imageview = (ImageView) findViewById(R.id.img_intro_logo);
    Animation anim = AnimationUtils.loadAnimation(this, R.anim.alpha);
    imageview.startAnimation(anim);
  }
}
</pre>

<br>

# 객체 움직임

객체 움직임을 나타내는 Tween Animation 예제를 살펴보겠습니다.

## scale.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;scale xmlns:android="http://schemas.android.com/apk/res/android"
  android:duration="3000"
  android:fromXScale="1.0"
  android:fromYScale="1.0"
  android:interpolator="@android:anim/accelerate_interpolator"
  android:pivotX="50%p"
  android:pivotY="50%p"
  android:toXScale="0.5"
  android:toYScale="0.5" /&gt;
</pre>

<br>


# set 태그 사용법

`set` 태그는 여러 종류의 Animation을 하나의 세트로 묶어서 동작하도록 하는 태그입니다. 예제는 다음과 같습니다.

## set1.xml (첫 번째 예제)

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;set xmlns:android="http://schemas.android.com/apk/res/android"
  android:interpolator="@android:anim/accelerate_interpolator"&gt;
  &lt;alpha
    android:duration="5000"
    android:fromAlpha="0.0"
    android:toAlpha="1.0" /&gt;

  &lt;scale
    android:duration="3000"
    android:fromXScale="1.0"
    android:fromYScale="1.0"
    android:pivotX="50%p"
    android:pivotY="50%p"
    android:startOffset="3000"
    android:toXScale="0.5"
    android:toYScale="0.5" /&gt;
&lt;/set&gt;
</pre>

`startOffset`은 시작 시간을 의미합니다. 위의 Animation 내용을 살펴보면, Animation이 시작되면서 5 초(5000 msec)동안 투명도가 서서히 변하는 애니메이션이 구동됩니다. 그리고, 3 초 후에 크기 변경 애니메이션이 3 초동안 실행됩니다. 즉, Animation 효과들이 순차적 또는 동시에 실행되도록 할 수 있습니다.

<br>

## set2.xml (두 번째 예제)

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?>
&lt;set xmlns:android="http://schemas.android.com/apk/res/android"
  android:interpolator="@android:anim/accelerate_interpolator"&gt;
  &lt;alpha
    android:duration="5000"
    android:fromAlpha="0.0"
    android:toAlpha="1.0" /&gt;

  &lt;scale
    android:duration="3000"
    android:fromXScale="1.0"
    android:fromYScale="1.0"
    android:pivotX="50%p"
    android:pivotY="50%p"
    android:startOffset="3000"
    android:toXScale="0.5"
    android:toYScale="0.5" /&gt;

  &lt;translate
    android:duration="3000"
    android:fromXDelta="0"
    android:fromYDelta="0"
    android:startOffset="5000"
    android:toXDelta="0"
    android:toYDelta="50%p" /&gt;

  &lt;rotate
    android:duration="1000"
    android:fromDegrees="0"
    android:pivotX="30%p"
    android:pivotY="50%p"
    android:startOffset="7000"
    android:toDegrees="360" /&gt;

&lt;/set&gt;
</pre>