---
layout: post
title: StringBuffer vs StringBuilder
category: Android
tag: [Android]
---

안드로이드 개발을 하면서 StringBuffer 또는 StringBuilder를 많이 사용하게 될 것입니다.
특히 개발하면서 로그(Log) 메세지 등을 길게 붙여서 만들 때,
 StringBuffer나 StringBuilder를 모르는 경우 다음과 같이 코드를 작성하는 경우가 많습니다.

<pre class="prettyprint">String strLog;
void log(String message) {
  strLog = strLog + message + "\n";
}</pre>
<br>

## String

String은 Immutable 하기 때문에 한 번 할당되면 메모리 공간이 변하지 않습니다.
무슨 말이냐면, 위의 예제에서 strLog 라는 변수에 'strLog + message + "\n"' 이라는 값을 넣게 되면,
새로운 String 인스턴스가 생기고 기존의 String은 제거가 되게 됩니다.
(정확히는 가비지 컬렉터(Garbage Collector)가 제거할 것입니다.)
즉, 메모리 할당과 삭제가 빈번하게 일어나기 때문에 성능 하락의 원인이 될 수 있습니다.

이런 경우에는 String 대신 StringBuffer나 StringBuilder를 사용하면 성능 향상에 도움이 됩니다.
두 클래스는 append() 메소드를 이용하여 기존에 할당받은 메모리 공간을 유연하게 늘리면서
사용을 하게 됩니다. 그래서 메모리 할당과 삭제가 빈번하게 일어나는 일을 방지할 수 있습니다.

그러면 StringBuffer와 StringBuilder의 차이점을 알아보도록 하겠습니다.

<br>

## StringBuffer vs StringBuilder

StringBuffer와 StringBuilder는 겉으로 보기에 크게 차이가 없습니다.
다만, 내부적으로 StringBuffer는 synchronized 키워드가 있어서 멀티쓰레드(Multi-Thread) 환경에서
좀 더 안전하다는 장점이 있습니다. 대신 성능은 StringBuilder 보다 약간 떨어지겠죠?

즉, 멀티쓰레드 환경에서는 StringBuffer, 그 외에는 StringBuilder를 사용하면 효율적으로
사용할 수 있습니다. 두 개를 외우기 싫으면 그냥 StringBuffer를 사용하시면 됩니다.
