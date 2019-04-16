---
layout: post
title: Kotlin 인터페이스
category: Kotlin
tag: [Kotlin]
---

## interface

코틀린에서는 Java에서와 달리 `interface` 내부의 함수가 내용을 가질 수도 있고, abstract 멤버 변수를 가질 수도 있습니다.

또한 Java에서는 `implements` 키워드를 이용해서 인터페이스를 구현했는데, 코틀린에서는 상속과 마찬가지로 콜론(`:`)을 사용해서 인터페이스를 구현할 수 있습니다.

<pre class="prettyprint">
interface OnEventListener {
    open fun onEvent(type: Int)
}

open class EventHandler(var name: String) : OnEventListener {
    override fun onEvent(type: Int) {
        TODO("not implemented")
    }
}
</pre>

<br>

## 다중 인터페이스 구현할 경우

아래 예제는 하나의 클래스가 여러 개의 인터페이스를 구현한 예제입니다. 인터페이스 내의 함수들이 구현체가 있고, 중복된 함수가 있을 경우 구현한 클래스에서는 필요한 클래스의 `super`를 호출할 수 있습니다. 둘 다 호출해도 상관없고, 필요한 `super`만 호출해도 상관없습니다.

<pre class="prettyprint">
interface onEventListener {
    fun onEvent() {}
    fun onMessageArrived() {}
}

interface onClientEventListener {
    fun onEvent() {}
    fun onConnected() {}
    fun onDisconnected() {}
}

class TcpServer : onEventListener, onClientEventListener {
    override fun onEvent() {
        super&lt;onEventListener&gt;.onEvent()
        super&lt;onClientEventListener&gt;.onEvent()
    }
}
</pre>