---
layout: post
title: for 문과 when 문
category: Kotlin
tag: [Kotlin]
---

## for 키워드

1 부터 10까지의 반복문은 다음과 같습니다. (1 <= i < 10)

<pre class="prettyprint">
for(i in 1..9) {
    println(i)
}
</pre>

또는 다음과 같이 사용할 수 있습니다.

<pre class="prettyprint">
for(i in 1 until 10) {
    println(i)
}
</pre>


만약 9 부터 1까지 거꾸로 내려가고 싶을 때는 다음과 같이 `downTo` 키워드를 사용합니다.

<pre class="prettyprint">
for(i in 9 downTo 1) {
    println(i)
}
</pre>

만약 증가폭을 특정 숫자만큼 하고 싶을 때는 `step` 키워드를 사용합니다.

<pre class="prettyprint">
for(i in 1 until 10 step 2) {
    println(i)
}
</pre>

`ArrayList` 등의 리스트를 사용할 경우 다음과 같이 `for` 문을 작성할 수 있습니다.

<pre class="prettyprint">
fun test() {
    val list = ArrayList&lt;String&gt;()

    list.add("Hello")
    list.add("Nice to meet you")
    list.add("Good bye")

    for (i in 0 until list.size) {
        println(list[i])
    }
}
</pre>

만약 `iterator`를 사용할 경우 다음과 같이 코드를 작성할 수 있습니다.

<pre class="prettyprint">
fun test() {
    val list = ArrayList&lt;String&gt;()

    list.add("Hello")
    list.add("Nice to meet you")
    list.add("Good bye")

    for (text: String in list) {
        println(text)
    }
}
</pre>

<br>

## when 키워드

`when`은 Java에서의 `switch`와 사용법이 유사합니다. 

<pre class="prettyprint">
fun test(menu: Int) {
    when (menu) {
        R.id.menu_start -> {

        }
        R.id.menu_stop -> {
            
        }
    }
}
</pre>

`break` 구문 없이 중괄호({}) 만으로 분기를 나눌 수 있습니다. 중괄호 없이 사용하더라도 다음 `->` 구문이 나올 때까지 실행합니다.

`when` 문 안에서 간단한 연산도 가능합니다.

<pre class="prettyprint">
fun main(args: Array&lt;String&gt;) {
    test(2)
    test(10)
    test(4)
    test(1)
}

fun test(menu: Int) {
    when {
        menu <= 3 -> {
            println("$menu <= 3")
        }
        menu <= 7 -> {
            println("3 < $menu <= 7")
        }
        else -> {
            println("$menu > 7")
        }
    }
}
</pre>