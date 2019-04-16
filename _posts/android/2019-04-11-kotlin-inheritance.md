---
layout: post
title: Kotlin 상속
category: Android, Kotlin
tag: [Android, Kotlin]
---

## 부모 클래스 구현하기

코틀린에서 상속 받을 때는 `:` 기호를 이용해서 상속 받습니다. 그리고 부모 클래스는 `open` 이나 `abstract` 키워드를 사용해야만 상속할 수 있습니다.

그리고 기본적으로 모든 클래스는 Java에서의 `Object` 처럼 `Any`라는 클래스를 상속받고 있습니다.

<pre class="prettyprint">
open class Shape(vertex: Int) {
    open fun onDraw(canvas: Canvas) {
    }
}

class Triangle() : Shape(3) {
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        TODO("Draw Lines")
    }
}

class Rectangle() : Shape(4) {
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        TODO("Draw Lines")
    }
}
</pre>

`open` 키워드를 사용하지 않은 경우는 코틀린에서 기본적으로 `final`로 선언됩니다. 즉 오버라이드를 하고 싶으면 반드시 `open`을 붙여야 합니다.

<br>

## 상속 받는 방법

부모 클래스의 생성자가 여러 개인 경우 상속받는 자식 클래스는 다음과 같이 작성할 수 있습니다.

<pre class="prettyprint">
class CustomDialog(ctx: Context) : AlertDialog(ctx) {
}
</pre>

또는 

<pre class="prettyprint">
class CustomDialog : AlertDialog {
    constructor(ctx: Context) : super(ctx)
}
</pre>

 와 같이 작성할 수 있습니다. 이 경우 두 번째 방법이 더 좋습니다.

 첫 번째 방법의 경우 부모 클래스의 디폴트 생성자가 정해져 있어서 다른 생성자를 만들기 어렵습니다.

 만약 다음과 같이 코드를 작성하면 `Primary constructor call expected` 오류가 발생합니다.

 <pre class="prettyprint">
class CustomDialog(ctx: Context) : AlertDialog(ctx) {
    constructor(ctx: Context, themeId: Int) : super(ctx, themeId)
}
</pre>

두 번째 방법으로 사용하면 다음과 같이 해결할 수 있습니다.

 <pre class="prettyprint">
class CustomDialog : AlertDialog {
    constructor(ctx: Context) : super(ctx, android.R.style.Theme_NoTitleBar)

    constructor(ctx: Context, themeId: Int) : super(ctx, themeId)
}
</pre>