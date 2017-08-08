---
layout: post
title: 최근 실행 이력(Multitask)에서 내 앱 숨기기
category: Android
tag: [Android]
---

안드로이드에서 멀티태스킹 버튼을 누르면 최근 실행한 앱들 이력이 좌르르 나옵니다.

여기에 표시되는 내 앱이 표시되지 않도록 하는 방법은 다음과 같습니다.

`AndroidManifest.xml` 파일의 액티비티 속성에 다음과 같은 항목을 추가해주면 됩니다.

<pre class="prettyprint">
&lt;activity
      android:label="@string/app_name"
      android:excludeFromRecents="true"&gt;
&lt;/activity&gt;
</pre>