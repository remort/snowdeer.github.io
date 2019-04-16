---
layout: post
title: 코틀린 함수(fun)
category: Kotlin
tag: [Kotlin]
---

## 함수 선언

함수의 기본 형태는 다음과 같습니다.

<pre class="prettyprint">
fun add(x: Int, y: Int): Int {
    return x + y
}
</pre>

위의 함수는 다음과 같이 조금씩 변형해서 사용할 수 있습니다.

<pre class="prettyprint">
fun add(x: Int, y: Int): Int = x + y
</pre>

<pre class="prettyprint">
fun add(x: Int, y: Int) = x + y
</pre>

<br>

## 변수 타입

코틀린은 자동 변환 기능을 허용하지 않습니다. 예를 들어 Java에서는 `int`에서 `double`이나 `float` 등으로 자동 변환이 가능했지만, 코틀린에서는 오류가 발생합니다. 다음과 같이 타입 변환을 항상 해야 합니다.

<pre class="prettyprint">
val i:Int = 1
val value:Long = i.toLong()
</pre>

코틀린은 `|`, `&`와 같은 비트 연산을 `or` 또는 `and`로 표현해야 합니다.

변수 추론 기능이 있어서 타입 선언을 생략하고 사용할 수 있다.

<pre class="prettyprint">
val i = 1
val hex = 0x23
val long = 4L
val double = 5.0
val float = 6.0F
val str = "Hello"
</pre>

## vararg

여러 개의 매개변수를 사용하고 싶을 때는 `vararg` 키워드를 사용합니다.

<pre class="prettyprint">
fun sum(vararg numbers: Int): Int {
    var ret = 0
    for (n in numbers)
        ret += n

    return ret
}

fun main(args: Array&lt;String&gt;) {
    println(sum(1))
    println(sum(1, 2, 3))
}
</pre>