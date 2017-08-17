---
layout: post
title: ListView의 구분선 삭제하기
category: Android
tag: [Android, UX]
---
# ListView의 구분선 삭제

ListView에서 각 칸을 구분하는 구분선 삭제는 XML에서 ListView의 속성에 다음 속성을 추가하면 됩니다.

<pre class="prettyprint">
android:divider="@null"
android:dividerHeight="0dp"
</pre>

<br>

## ListView 예제 

<pre class="prettyprint">
  &lt;ListView
    android:id="@+id/listview"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:layout_above="@id/layout_input"
    android:divider="@null"
    android:dividerHeight="0dp" /&gt;
</pre>