---
layout: post
title: Kotlin 멤버 변수 get/set
category: Android
tag: [Android, Kotlin]
---

코틀린에서는 변수 선언만 하면 컴파일러가 자동으로 `get/set` 함수를 생성해줍니다.

<pre class="prettyprint">
class Person() {
    var name: String = ""
    var age: Int = 0
}

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        var p = Person()
        p.name = "snowdeer"
        p.age = 30
    }
}
</pre>

<br>

## get/set 함수 오버라이딩

<pre class="prettyprint">
class Person() {
    var name: String = ""
        set(name) {
            field = "[-- $name --]"
        }
        get() = "Hello, " + field


    var age: Int = 0
}

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        var p = Person()
        p.name = "snowdeer"
        p.age = 30

        Log.i("snowdeer", "snowdeer] " + p.name)
    }
}
</pre>

실행 결과는 다음과 같습니다.

<pre class="prettyprint">
Hello, [-- snowdeer --]
</pre>

만약 외부에서 `set` 함수에 접근하지 못하게 하려면 `private` 키워드를 사용하면 됩니다.

<pre class="prettyprint">
class Person() {
    var name: String = ""
        private set(name) {
            field = "[-- $name --]"
        }
        get() = "Hello, " + field
}
</pre>