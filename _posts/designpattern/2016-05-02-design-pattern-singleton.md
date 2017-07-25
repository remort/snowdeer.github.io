---
layout: post
title: 싱글톤(Singleton) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

프로그램에서 단 하나의 인스턴스(Instance)만 존재하게 하고 싶을 때 사용하는 패턴입니다.
어디서든지 그 인스턴스에 접근할 수 있기 때문에 전역 변수 등을 관리할 때 상당히 편리하기
때문에 많은 사람들이 남발하고 있는 패턴이기도 합니다.

하지만, 싱글톤(Singleton)은 실제로는 단순 전역 변수와 거의 같은 용도로 많이 쓰이며
그 성격상 객체 지향과는 거리가 있는 패턴입니다. 프로그램 어느 곳에서든
Singleton에 접근할 수 있기 때문에 정보의 보호도 되지 않으며, 누가 어떤 값을 건드렸는지
추적하기도 쉽지 않습니다. 따라서 가급적 사용하지 않는 것이 좋지만 적절하게 제한적을
사용하면 편리하긴 합니다.
<h3>UML</h3>

싱글톤 패턴의 UML은 다음과 같다. 달랑 클래스 하나 뿐이기 때문에 단순합니다.

![Image]({{ site.baseurl }}/assets/design-patterns/singleton.gif)

<br>
## 예제 코드
싱글톤 패턴을 코드로 구현하면 다음과 같습니다.
(다만, 이 코드는 멀티쓰레드(Multi-Thread) 환경에서 문제가 발생하는 코드입니다.
해결법이 여러가지인데, 단계별로 해결된 코드를 설명해나가도록 하겠습니다.)

<pre class="prettyprint">public class Singleton {

  private static Singleton mInstance = null;

  private Singleton() {
  }

  public static Singleton getInstance() {
    if(mInstance == null) {
      mInstance = new Singleton();
    }

    return mInstance;
  }
}
</pre>
<br>

위 코드는 멀티쓰레드 환경에서 getInstace() 메소드가 끝나기 전에 여러 쓰레드에서
동시에 접근을 할 수 있기 때문에 재수가 없으면 Singleton 인스턴스가 여러 개 생성될 수 있습니다.

이 경우 getInstance() 메소드를 synchronized로 동기화시켜 해결할 수 있습니다.
<pre class="prettyprint">public class Singleton {

  private static Singleton mInstance = null;

  private Singleton() {
  }

  public synchronized static Singleton getInstance() {
    if(mInstance == null) {
      mInstance = new Singleton();
    }

    return mInstance;
  }
}</pre>
<br>
하지만, 함수 전체에 synchronized는 동기화 과정에서 속도 문제가 발생하고 성능 저하를
가져 올 수 있습니다. (물론, 위 예제 코드 정도로는 그렇게 치명적인 속도 문제가 발생할 가능성은 많지 않습니다.)

그래서 함수 내부에 최소 구간에만 synchronized를 거는 방법도 있습니다.
<pre class="prettyprint">public class Singleton {

  private static Singleton mInstance = null;

  private Singleton() {
  }

  public static Singleton getInstance() {
    if(mInstance == null) {
      synchronized(Singleton.class) {
        if(mInstance == null) {
          mInstance = new Singleton();
        }
      }
    }

    return mInstance;
  }
}</pre>
<br>
그리고 코드를 좀 더 깔끔하고 쉽게 구현하기 위해서는 아예 처음부터 인스턴스를 생성해버리는
방법도 있습니다. 저는 처음부터 인스턴스를 생성하는 방법을 가장 많이 사용하고 있습니다.
구현도 쉽고 코드도 더 깔끔한 것 같아서입니다.
<pre class="prettyprint">public class Singleton {

  private static Singleton mInstance = new Singleton();

  private Singleton() {
  }

  public static Singleton getInstance() {
    return mInstance;
  }
}</pre>

물론 이 경우는 프로그램이 처음 실행되면서 바로 인스턴스가 생겨버리기 때문에,
불필요한 부분에서 인스턴스가 메모리를 차지해버린다는 단점이 있습니다. 하지만, 그런 경우는
Singleton을 사용하지 않는게 더 적합한 경우가 많기 때문에 대부분의 Singleton 구현은
위의 마지막 예제처럼 구현하면 됩니다.

하지만 최선은 싱글톤 패턴을 사용하지 않거나 최소한으로 사용하는 거라고 생각합니다.
