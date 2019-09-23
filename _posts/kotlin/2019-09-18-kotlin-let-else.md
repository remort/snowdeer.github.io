---
layout: post
title: let 명령어에 else 처리하기
category: Kotlin
tag: [Kotlin]
---

# 예제 코드

<pre class="prettyprint">
fun main(args: Array&lt;String&gt;) {
    var str1: String? = "hello"
    var str2: String? = null

    str1?.let {
        println("str1(1): $str1")
    } ?: run {
        println("str1(2): $str1")
    }

    str2?.let {
        println("str2(1): $str2")
    } ?: run {
        println("str2(2): $str2")
    }
}
</pre>

결과값은 다음과 같습니다.

<pre class="prettyprint">
str1(1): hello
str2(2): null
</pre>