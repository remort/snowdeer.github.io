---
layout: post
title: Kotlin 생성자
category: Kotlin
tag: [Kotlin]
---

## constructor 키워드

기본적으로 코틀린에서 클래스 생성자는 다음과 같이 클래스 선언부에서 `constructor` 키워드를 시용해서 만들어줍니다.

<pre class="prettyprint">
class Person constructor(name: String, age: Int)
</pre>

위와 같이 클래스 선언부에서 생성자를 사용할 때는 `constructor` 키워드를 생략할 수 있습니다.

<pre class="prettyprint">
class Person(name: String, age: Int)
</pre>

<br>

## 생성자를 여러 개 선언할 경우

<pre class="prettyprint">
class Person() {
    constructor(name: String, age: Int) : this()

    constructor(name: String) : this()
}
</pre>

<pre class="prettyprint">
class Person(name: String) {
    constructor(name: String, age: Int) : this(name)
}
</pre>

와 같은 형태로 사용할 수 있습니다. 생성자 뒤에 `this` 함수 호출하는 부분은 필수입니다.

<br>

## 생성자 매개 변수의 기본값 설정

<pre class="prettyprint">
class Person(name: String = "default", age: Int = 0)
</pre>

<br>

## 생성자 함수 바디

클래스에서 변수의 값 선언 외에 별도 처리가 필요한 경우는 `init {}` 함수를 사용해서 처리할 수 있습니다.

<pre class="prettyprint">
class Person(var name: String?, val age: Int = 0) {
    init {
        if (name.isNullOrEmpty()) {
            name = "snowdeer"
        }
    }
}
</pre>

생성자 매개 변수에서 `val`, `var` 등의 선언이 생략되면 기본적으로는 `val`로 인식이 됩니다.

<br>

## 접근 제한자

* `private`: 클래스 내부에서만 접근 가능
* `protected`: 상속받은 클래스에서만 접근 가능
* `internal`: 같은 모듈 안에서만 접근 가능
* 생략: 생략된경우 `public`로 간주됨