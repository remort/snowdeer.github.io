---
layout: post
title: 안드로이드 상태바 투명 처리
category: Android
tag: [Android, UX]
---

안드로이드 상태바를 투명하게 만드는 방법입니다.

대충 안드로이드 상태바는 다음과 같은 형태로 표현할 수 있습니다.
<br>

## 1) 아무런 설정을 안 해줬을 때의 기본적인 상태바
![image -fullwidth]({{ site.baseurl }}/assets/2017-04-14-android-transparent-status-bar/01.png)
<Br>
## 2) 색상을 입힌 상태바
![image -fullwidth]({{ site.baseurl }}/assets/2017-04-14-android-transparent-status-bar/02.png)
<br>
## 3) 투명 처리를 한 상태바
![image -fullwidth]({{ site.baseurl }}/assets/2017-04-14-android-transparent-status-bar/03.png)

과거에는 완전히 투명한 상태바도 표현할 수 있었는데, 지금은 반투명 상태로 표현되고 있습니다.

<br>

일단, 반투명 상태의 상태바는 다음과 같은 테마를 적용하여 구현할 수 있습니다.

<br>
## styles.xml
<pre class="prettyprint">&lt;style name="AppTheme" parent="Theme.AppCompat.Light.NoActionBar"&gt;
  &lt;!-- Customize your theme here. --&gt;
  &lt;item name="colorPrimary"&gt;@color/colorPrimary&lt;/item&gt;
  &lt;item name="colorPrimaryDark"&gt;@color/colorPrimaryDark&lt;/item&gt;
  &lt;item name="colorAccent"&gt;@color/colorAccent&lt;/item&gt;
  &lt;item name="android:windowDrawsSystemBarBackgrounds"&gt;true&lt;/item&gt;
  &lt;item name="android:statusBarColor"&gt;@color/transparent&lt;/item&gt;
  &lt;item name="android:windowTranslucentStatus"&gt;true&lt;/item&gt;

&lt;/style&gt;</pre>
<br>

때에 따라서 반투명 상태보다는 색상을 입힌 상태바가 더 좋을 때가 있습니다.
상태바의 색상을 primaryColor와 똑같이 지정하면 투명 상태처럼 보이기도 합니다. 그럴 때는
<pre class="prettyprint">&lt;style name="AppTheme" parent="Theme.AppCompat.Light.NoActionBar"&gt;
  &lt;!-- Customize your theme here. --&gt;
  &lt;item name="colorPrimary"&gt;@color/colorPrimary&lt;/item&gt;
  &lt;item name="colorPrimaryDark"&gt;@color/colorPrimaryDark&lt;/item&gt;
  &lt;item name="colorAccent"&gt;@color/colorAccent&lt;/item&gt;
  &lt;item name="android:windowDrawsSystemBarBackgrounds"&gt;true&lt;/item&gt;
  &lt;item name="android:statusBarColor"&gt;@color/colorPrimary&lt;/item&gt;
  &lt;item name="android:windowTranslucentStatus"&gt;false&lt;/item&gt;

&lt;/style&gt;</pre>
와 같이 지정해주면 됩니다.
