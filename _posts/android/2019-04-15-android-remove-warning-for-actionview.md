---
layout: post
title: consider adding at least one Activity with an ACTION-VIEW intent filter
category: Android
tag: [Android, Kotlin]
---

`AndroidManifest.xml` 에서 다음과 같은 오류가 발생할 경우 해결하는 방법입니다.

> App is not indexable by Google Search; consider adding at least one Activity with an ACTION-VIEW intent filter.

적어도 하나 이상의 액티비티의 인텐트 필터에 다음 라인을 추가하면 됩니다.

<pre class="prettyprint">
&lt;action android:name="android.intent.action.VIEW"/&gt;
</pre>