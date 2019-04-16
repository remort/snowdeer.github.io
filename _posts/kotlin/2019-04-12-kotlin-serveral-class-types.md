---
layout: post
title: 다양한 클래스 타입
category: Kotlin
tag: [Kotlin]
---

## data class

<pre class="prettyprint">
data class LogItem(val text: String, val timestamp: String)
</pre>

data 클래스는 데이터만을 갖는 클래스입니다. 반드시 디폴트 생성자를 선언해야 하며, 인자는 `val` 또는 `var` 키워드를 꼭 사용해야 합니다. 컴파일러는 자동으로 `equal()` 및 `toString()` 함수를 생성해줍니다.

<br>

## enum class

코틀린에서 `enum`은 다음과 같은 형태로 선언할 수 있습니다.

<pre class="prettyprint">
enum class COLOR(val rgb: Int) {
    RED(0xFF0000),
    GREEN(0x00FF00),
    BLUE(0x0000FF),
}

enum class STATUS(val text: String) {
    STARTED("Started"),
    FINISHED("Finished"),
    FAILED("Failed"),
    UNKNOWN("Unknown"),
}
</pre>

사용 방법은 다음과 같습니다.

<pre class="prettyprint">
var color = COLOR.BLUE
println(color)
println(color.rgb)

var status = STATUS.STARTED
println(status)
println(status.text)
</pre>

<br>

## sealed class

sealed 클래스는 프로그램 내부에서는 상속을 할 수 있지만, 외부 모듈에서는 상속을 할 수 없도록 하는 키워드입니다.
다음과 같이 사용할 수 있습니다.

<pre class="prettyprint">
sealed class SnowSDK {
}
</pre>

같은 프로젝트 안에만 있다면 sealed 클래스를 상속한 클래스가 어느 파일에 위치하든지 상관이 없습니다.

<br>

## object 타입

익명(anonymous) 클래스를 사용할 때 Java에서는 다음과 같이 사용했습니다.

<pre class="prettyprint">
button.setOnClickListener(new View.OnClickListener() {
  @Override
    public void onClick(View v) {
    // TODO   
  }
});
</pre>

하지만 코틀린에서는 다음과 같이 `object` 키워드를 이용해서 익명 클래스를 사용할 수 있습니다.

<pre class="prettyprint">
button.setOnClickListener(object : View.OnClickListener() {
    fun onClick(v: View) {
        TODO("...")
    }
})
</pre>
