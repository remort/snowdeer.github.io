---
layout: post
title: 람다(Lambda)
category: Kotlin
tag: [Kotlin]
---

람다 함수는 다음과 같은 형태로 사용할 수 있습니다.

<pre class="prettyprint">
(매개 변수) -> { TODO("...) }
</pre>

실제 코드로 예를 들면 다음과 같습니다.

<pre class="prettyprint">
button.setOnClickListener((v) -> {
    TODO("onClicked !!")
})
</pre>