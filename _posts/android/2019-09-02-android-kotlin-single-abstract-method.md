---
layout: post
title: SAM(Single Abstract Method)
category: Kotlin
tag: [Android, Kotlin]
---

# SAM 변환

코틀린은 자바에서 작성한 인터페이스에 대해 SAM(Single Abstract Method) 변환을 지원합니다. 그래서 인터페이스를 매개변수로 받는
함수에 대해서 인터페이스 대신 함수를 전달할 수 있고 코드가 간결해집니다.

대표적인 예제로 `View.SetOnClickListener()`을 들 수 있습니다.

<br>

## View.SetOnClickListener 예제 (Java)

<pre class="prettyprint">
button.setOnClickListener(new View.OnClickListener() {

    @Override
    public void onClick(View v) {
        // TODO
    }
});
</pre>

<br>

## View.SetOnClickListener 예제 (Kotlin)

<pre class="prettyprint">
button.setOnClickListener(object: View.OnClickListener {
    override fun onClick(v: View) {
        // TODO
    }
})
</pre>

<br>

## 람다를 이용해 인터페이스 대신 함수를 전달하는 코드 (Kotlin)

<pre class="prettyprint">
button.setOnClickListener({ v: View ->
    Unit
    // TODO
})
</pre>

<br>

## SAM 변환 후 간소화된 코드 (Kotlin)

<pre class="prettyprint">
button.setOnClickListener {
    // TODO
}
</pre>

와 같은 형태가 됩니다. 하나의 함수만을 포함하는 인터페이스는 이와 같이 단순하게 표현할 수 있습니다.