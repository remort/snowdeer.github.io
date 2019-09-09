---
layout: post
title: 코틀린의 철학
category: Kotlin
tag: [Kotlin]
---

# 코틀린의 주요 특성

### 정적 타입 언어

코틀린은 정적 언어이기 때문에 컴파일 타임에 모든 객체와 메소드의 타입을 알 수 있습니다. 코틀린은 타입 추론(Type Inference)을 지원하고 있는데,
역시 컴파일 시점에 타입을 확정하고 검증합니다.

### 함수형 프로그래밍

코틀린은 함수형 프로그래밍을 지원합니다. 함수를 변수에 담을 수 있고, 파라메터 등으로 전달도 가능합니다. 
Java의 경우도 Java8 부터 어느 정도 지원하긴 합니다.

<br>

# 코틀린의 철학

### 실용성

코틀린은 기존의 Java의 불편한 점을 개선하고 간소화하는 방향으로 만들어진 실용적인 언어입니다.

* Null에 대한 고민이 많이 사라졌습니다.
* 타입 추론으로 코드가 간결해졌습니다.
* `var`, `val`로 개발 실수를 많이 줄여줍니다.

### 간결성

코틀린은 기존 코드의 복잡하고 긴 코드들을 간소화할 수 있습니다. 예를 들어 `getter`와 `setter`를 컴파일 시점에 자동으로
생성할 수도 있습니다. `Nulltype` 체크를 간결하게 할 수 있고, 다양한 람다(Lambda) 함수를 간결한 문법으로 사용할 수 있습니다.

`Data Binding`의 경우도 기존의 View를 Binding하는 방법보다 간결합니다.

### 안정성

`NullPointerException`을 컴파일 시점에 잡아 줄 수 있기 때문에 보다 쉽게 안정적인 코드를 작성할 수 있습니다.

### 상호 운용성

기존에 Java에서 사용하던 코드나 라이브러리들을 코틀린에서 그대로 사용할 수 있습니다. 또한 혼용해서 사용도 가능하며, 컴파일하고 나면
Java와 같이 `*.class` 파일이 생성됩니다.
반대로 코틀린에서 작성한 코드를 Java에서 대부분 사용가능합니다. (100%는 아니고 특정 경우엔 약간의 코드 수정이 필요하기도 합니다.)

<br>

# Java 대비 코틀린의 새로운 부분들

### 제어구조 식

`if`, `when`, `try/catch` 등의 제어 구조가 식으로 될 수 있습니다.

<pre class="prettyprint">
val a = 1
val b = 2
val c = 3

val ret = a + b + if(c == 3) c else 0
</pre>

### when

`when` 명령어는 Java의 `switch`의 확장 개념입니다. 객체끼리 비교도 가능하며 구문이 아니라 식으로 사용도 가능합니다.
다음 예제와 같은 형태로도 사용 가능합니다.

<pre class="prettyprint">
fun cases(obj: Any) = 
    when(obj) {
        1 -> "One"
        "hello" -> "snowdeer"
        is Long -> "obj is Long."
        else -> "Unknown"
    }
</pre>

### Null Safety

`?`를 이용해서 `Null`이 될 수 있는 타입과 될 수 없는 타입을 명시적으로 구분합니다.

### 확장 함수

어떤 클래스를 상속받지 않더라도 특정 클래스에 메소드를 추가할 수 있습니다. 

<pre class="prettyprint">
fun String.getCenterChar() : Char = 
    this.get(this.lastIndex/2)

fun test() {
    println("Hello, snowdder".getCenterChar())
}
</pre>