---
layout: post
title: 코틀린 확장 함수
category: Kotlin
tag: [Kotlin]
---

코틀린은 확장 함수를 지원합니다. 

Java의 경우 기존에 만들어진 클래스에 새로운 메소드를 추가하려면 해당 클래스를 상속하는 새로운 클래스를 만들어야 하는데, 
코틀린에서는 확장 함수(Extension Function)를 이용해서 상속 없이 기존 클래스에 새로운 함수를 추가할 수 있습니다.

이 때 확장 함수를 추가할 대상 클래스는 리시버 타입(Receiver Type)이라고 합니다.

예를 들어 다음과 같은 방법으로 확장 함수를 추가할 수 있습니다.

<pre class="prettyprint">
fun main(args: Array&lt;String&gt;) {
    println("snowdeer".hello())
}

fun String.hello() : String {
    return "$this, hello"
}
</pre>

확장 함수의 사용 방법은 클래스 내 정의된 메소드와 동일하게 점(.)을 찍고 호출할 수 있지만, 
실제로는 클래스 외부에서 추가된 함수이기 때문에 함수 내에서는 `private`나 `protected`로 선언된 변수나 메소드에 접근할 수 없습니다.

