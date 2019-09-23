---
layout: post
title: 코루틴(Coroutine) Dispatchers.Main 사용하기
category: Kotlin
tag: [Android, Kotlin]
---

# Dispatchers.Main

`Dispatchers.Main`는 해당 `CoroutineScope`을 Main UI Thread에서 동작시키도록 합니다.

안드로이드에서 `Dispatchers.Main`을 그냥 사용하려고 하면 다음과 같은 에러가 발생하는데, 

<pre class="prettyprint">
java.lang.ILLegalStateException: Module with Main dispatcher is missing. Add dependency with required Main dispatcher, e.g. 'kotlinx-coroutines-android'
</pre>

`gradle`에 다음 라인을 추가해주면 됩니다.

<pre class="prettyprint">
implementation 'org.jetbrains.kotlinx:kotlinx-coroutines-android:1.3.1'
</pre>

`Dispatchers.Main`을 이용해서 코루틴을 구동시키면 Main UI Thread에서 동작하기 때문에, 
`Thread.currentThread().id` 값을 확인해보면 같은 `id` 값을 가지는 것을 확인할 수 있습니다. 

또한 시간이 오래 걸리는 작업 등을 동작시킬 경우 `ANR(Application Not Responding)`과 같은 오류가 발생할 수도 있으니 주의해야 합니다.