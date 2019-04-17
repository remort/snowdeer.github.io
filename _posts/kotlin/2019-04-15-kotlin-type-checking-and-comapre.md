---
layout: post
title: 타입 체크 및 캐스팅, 비교 연산
category: Kotlin
tag: [Kotlin]
---

## is와 as

코틀린에서는 타입 체크할 때 `is` 키워드를 사용하며, Java에서의 `instanceof`와 동일한 역할을 합니다.
그리고 타입 캐스팅시에는 `as` 키워드를 사용합니다.

<pre class="prettyprint">
fun setLayoutParam(view: View) {
    if (view is LinearLayout) {
        var param = view.layoutParams as LinearLayout.LayoutParams
        param.gravity = Gravitiy.CENTER
        view.layoutParams = param
    } else if (view is RelativeLayout) {
        var param = view.layoutParams as RelativeLayout.LayoutParams
        param.addRule(RelativeLayout.ALIGN_PARENT_CENTER)
        view.layoutParams = param
    }
}
</pre>

<br>

## NullPoint Exception 방지

코틀린에서는 `?` 기호를 이용해서 해당 변수가 `null` 값을 가질 수 있음을 알려주며, 기본적으로는 `non null` 상태입니다.

<pre class="prettyprint">
var str:String? = "hello"
str = null
</pre>

그리고 변수를 사용할 때도 변수뒤에 `?` 기호를 붙여서 `Nullpoint Exception` 처리를 할 수 있습니다.

<pre class="prettyprint">
fun test() {
    var str: String? = "hello"
    str = null
    var length: Int? = str?.length
}
</pre>

위의 예제에서 만약 `str` 변수가 `null`이 되면 `str?.length`도 `null`을 리턴하기 때문에 `length` 변수가 `null`이 됩니다.
즉, 위험할 수 있는 코드이기 때문에 아래와 같이 표현할 수 있습니다.

<pre class="prettyprint">
fun test() {
    var str: String? = "hello"
    str = null
    var length: Int = str?.length ?: 0
}
</pre>

위와 같이 코드를 작성하면 `str?.length`가 `null`일 경우 숫자 0 으로 리턴하기 때문에 `length` 변수는 항상 값을 가질 수 있습니다.

그리고 `!!` 기호를 사용하게 되면 명시적으로 변수에 절대 `null`을 참조할 수 없다는 것을 지정할 수 있습니다.

<pre class="prettyprint">
fun test() {
    var str: String? = "hello"
    var length: Int = str!!.length
}
</pre>

<br>

## == 와 ===

`==`와 `===`는 비교 연산자로 `==`는 Java에서 사용하던 `==`와 동일한 역할을 합니다. 그리고 내부적으로 NULL 체크를 하기 때문에 좀 더 간략하게 코드를 작성할 수 있습니다. 다만, `a == b`에서 `a`와 `b` 모두 `null`인 경우 `true`가 되기 때문에 주의할 필요는 있습니다.

`===`의 경우는 두 변수가 정말 똑같은 주소값을 갖고 있는지 판단하며 Java에서의 `equals()`와 동일합니다.