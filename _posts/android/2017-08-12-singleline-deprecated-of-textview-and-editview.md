---
layout: post
title: TextView, EditView의 SingleLine 속성 Deprecated 해결
category: Android
tag: [Android, UX]
---
# SingleLine Deprecated

TextView나 EditView에서 한 줄만 표시되도록 하는 속성은 `singleLine`이었습니다. 하지만 지금은 Deprecated 된 속성입니다.

따라서 다른 방법을 찾아봐야 합니다.

다음과 같은 속성을 추가하면 기존 `singleLine`과 동일한 효과를 냅니다.

<pre class="prettyprint">
android:inputType="text"
android:maxLines="1"
</pre>

## EditText 예제 

<pre class="prettyprint">
&lt;EditText
  android:id="@+id/input"
  android:layout_width="0px"
  android:layout_height="wrap_content"
  android:layout_weight="4"
  android:inputType="text"
  android:maxLines="1" /&gt;
</pre>