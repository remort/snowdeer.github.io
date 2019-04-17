---
layout: post
title: lateinit 및 lazy()
category: Kotlin
tag: [Kotlin]
---

## lateinit

`lateinit`는 다음과 같은 특징이 있습니다.

* `var` 변수에만 사용 가능
* `Non-null` 데이터 타입에만 가능
* primitive type에는 사용할 수 없음
* 클래스 생성자에서 사용 못함
* 로컬 변수로 사용 못함

그럼, 어떤 경우에 사용해야 할까요? 코틀린은 NULL 체크에 대한 검사가 엄격하기 때문에 

<pre class="prettyprint">
class Sample {
    private var name: String
}
</pre>

와 같은 코드는 `Property must be initialized or be abstract` 오류가 발생합니다. 

실제로 초기화 때 값을 지정해주고 코드를 작성하는 것이 더욱 좋지만, 그렇지 못한 경우가 많이 발생합니다. 그럴 때는 아래와 같이 `lateinit` 키워드를 변수 앞에 적어주면 변수 초기화를 나중에 할 수 있게 됩니다.

<pre class="prettyprint">
class Sample {
    private lateinit var name: String
}
</pre>

<br>

## lazy 

`lazy()` 함수는 `lateinint`와 비슷한 역할을 하는데 다음과 같은 특징이 있습니다.

* `val` 변수에만 사용 가능
* primitive type에도 사용 가능
* `Non-null`, `Nullable` 둘 다 사용 가능
* 클래스 생성자에서 사용 못함
* 로컬 변수로 사용 가능

<pre class="prettyprint">
private val lazyExample: String by lazy {
    println("lazyExample - init()")

    "[lazy] lazyExample is initialized."
}

fun test() {
    println("Start...")

    println("1st : $lazyExample")
    println("2nd : $lazyExample")
    println("3rd : $lazyExample")
}
</pre>

실행 결과는 다음과 같습니다.

<pre class="prettyprint">
Start...
lazyExample - init()
1st : [lazy] lazyExample is initialized.
2nd : [lazy] lazyExample is initialized.
3rd : [lazy] lazyExample is initialized.
</pre>

위 결과와 같이, 처음 호출될 시점에 초기화를 한 번 하며 그 이후로는 그 결과값만 사용하는 것을 확인할 수 있습니다.