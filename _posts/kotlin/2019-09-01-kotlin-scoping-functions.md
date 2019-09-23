---
layout: post
title: 범위 지정 함수(let, apply, with, run, also)
category: Kotlin
tag: [Kotlin]
---

# 범위 지정 함수

범위 지정 함수는 중괄호({ })로 묶여 있는 부분에 전체적으로 적용되는 함수이며, 
코틀린에서는 `let`, `apply`, `with`, `run`, `also` 등과 같은 함수들을 제공하고 있습니다.

이 함수들은 전부 비슷한 기능을 하며, 사용법 또한 비슷합니다.

<br>

## let

`let`은 함수를 호출한 객체를 이어지는 블록의 매개변수로 전달하는 역할을 합니다. 

함수의 정의는 다음과 같이 되어 있습니다.

> fun <T, R> T.let(block: (T) -> R): R

<br>

예를 들면 다음과 같은 코드를 작성할 수 있습니다.

<pre class="prettyprint">
fun sample(message:String) {
    message.let {
        Log.i("message: $it")
    }
}
</pre>

`message`라는 매개변수를 `let`으로 다음 블럭에 넘기고, 그 안에서는 `it` 키워드를 이용해서 `message`를 사용할 수 있습니다.

다음과 같이 `?` 키워드와 같이 사용해서 Null Check 용도로도 사용할 수 있습니다.

<pre class="prettyprint">
fun sample(message:String?) {
    message?.let {
        Log.i("message: $it")
    }
}
</pre>

<br>

## apply

`apply`도 `let`과 사용법은 비슷하지만 블럭으로 넘긴 매개변수가 `it`이 아니라 `this`라는 점에서 차이가 있습니다.

함수의 정의는 다음과 같습니다.

> fun <T> T.apply(block: T.() -> Unit): T

<br>

<pre class="prettyprint">
private val timelineGuideLinePaint = Paint()

timelineGuideLinePaint.apply {
    color = context.getColor(R.color.colorTimelineGuideLine)
    style = Paint.Style.STROKE
    pathEffect = DashPathEffect(floatArrayOf(20F, 10F), 20F)
    strokeWidth = 1f
}
</pre>

<br>

## with

`with`도 `apply`와 비슷한 용도로 사용됩니다. 다만, `with`는 인자를 가지며, 해당 인자를 다음 함수 블럭으로 전달합니다.

함수 정의는 다음과 같습니다.

> fun <T, R> with(receiver: T, block: T.() -> R): R

<br>

<pre class="prettyprint">
fun sample() {
    with(textView) {
        text = "hello, snowdeer"
    }
}
</pre>

<br>

## run

`run` 함수는 인자없이 사용하는 익명 함수처럼 사용하는 방법과 객체에서 호출하는 형태를 제공하고 있습니다.

함수 정의는 다음과 같습니다.

> fun <R> run(block: () -> R): R

> fun <T, R> T.run(block: T.() -> R): R

<br>

<pre class="prettyprint">
val result = run {
    val a = getResult(100, 50)
    val b = getResult(1000, 2000)
    
    a + b
}

fun printName(person: Person) = person.run {
    print("${person.name}")
}

</pre>

`run` 내부에서 선언되는 변수들은 블록 외부에 노출되지 않기 때문에 변수 영역을 명확하게 분리할 수 있습니다

