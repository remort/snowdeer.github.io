---
layout: post
title: Collections
category: Kotlin
tag: [Kotlin]
---

코틀린은 자바에서와는 달리 리스트(List)나 맵(Map) 등의 자료 구조에 읽기 전용 객체와 수정이 가능한 객체로 나누어 놓았습니다. 

코틀린에서는 리스트를 생성할 때 클래스를 선언하지 않아도 만들 수 있는 함수를 제공하고 있습니다. 리스트를 생성하는 함수는 `listOf`이며, 맵을 생성하는 함수는 `mapOf`입니다. 이 함수를 이용하면 읽기 전용의 리스트가 만들어집니다.

<pre class="prettyprint">
fun init() {
    val numList = listOf(1, 3, 7, 5, 10)
    val strList = listOf("snowdeer", "ran", "yang")

    val map = mapOf(1 to "one", 2 to "two", 3 to "three")
}
</pre>

읽기 전용 리스트를 만들면 `get`, `first`, `last` 등 데이터를 읽을 수 있는 함수들만 제공됩니다.

만약, 수정이 가능한 리스트들을 만들고 싶으면 `mutableListOf`, `mutableMapOf` 함수를 사용하면 됩니다.

<pre class="prettyprint">
fun init() {
    val numList = mutableListOf(1, 3, 7, 5, 10)
    val strList = mutableListOf("snowdeer", "ran", "yang")
    val map = mutableMapOf(1 to "one", 2 to "two", 3 to "three")
}
</pre>

