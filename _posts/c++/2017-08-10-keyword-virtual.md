---
layout: post
title: 키워드 'virtual' 심화 탐구
category: C++
tag: [C++, CleanCode]
---

C++에서 자주 쓰이는 `virtual` 키워드에 대해서 알아보도록 하겠습니다. 

먼저 가장 대표적인 특징으로는 오버라이딩이 될 수 있습니다.

<br>

# 오버라이딩

~~~
상속시 `virtual` 함수만 정상적으로 오버라이딩(overriding)될 수 있습니다.
~~~

다음 예제를 살펴보도록 하겠습니다.

<pre class="prettyprint">
class Parent {

 public:
  void hello() {
    printf("Hello. This is the parent.\n");
  }
};

class Child: public Parent {
 public:
  void hello() {
    printf("Hello. This is the child.\n");
  }
};
</pre>

여기서 만약 다음 코드를 호출하면 결과는 어떤게 나올까요?

<pre class="prettyprint">
  Child c;
  c.hello();
</pre>

결과는 `Hello. This is the child.`가 화면에 출력될 것입니다. 결과만 봐서는 굳이 `virtual` 키워드를 사용하지 않더라도 정상적으로 오버라이딩된 것처럼 보입니다.

하지만, 만약 다음 코드처럼 부모 클래스의 참조를 이용해서 호출을 하면, 

<pre class="prettyprint">
  Child c;
  Parent& ref = c;
  ref.hello();
</pre>

결과는 `Hello. This is the parent.`가 나옵니다. 부모 클래스의 `hello()` 함수가 `virtual`이 아니기 때문에 이런 결과가 발생하게 됩니다.

즉, 단순하게 사용할 때는 `virtual` 키워드를 사용하지 않더라도 마치 정상적으로 오버라이딩된 것처럼 보였지만, 실제로는 각 함수들이 정상적으로 오버라이딩이 된 것이 아니라 부모의 함수와 자식의 함수가 각각 따로 존재했을 뿐이라는 것을 알 수 있습니다.

<br>

## vtable

`virtual` 키워드를 사용하지 않은 경우에는 부모의 함수와 자식의 함수가 각각 개별적으로 하드코딩되어져서 두 함수 모두 존재하게 됩니다. 이 때 함수의 이름은 컴파일 타임 타입에 맞춰져서 각각 다른 이름으로 저장이 됩니다.

하지만 `virtual` 키워드를 사용한 경우에는 해당 함수를 `vtable(Virtual Table)`이라고 부르는 특수한 메모리 영역에서 함수 주소값을 관리하도록 합니다.

즉, 함수가 실행될 때 `vtable`을 참조하여 실제로 올바르게 오버라이딩된 함수를 찾아서 실행해주기 때문에 `virtual` 키워드를 사용해야만 '정상적으로 오버라이딩 되었다'라고 할 수 있습니다.

<br>

## virtual 키워드에 대한 논쟁

그래서 많은 프로그래머들은 **모든 함수들을 `virtual`로 선언**하는 것을 권하고 있습니다. 그렇다면 애시당초 `virtual` 키워드를 사용하지 않더라도 디폴트로 `virtual`처럼 취급하면 되지 않냐는 의문점이 남습니다.

~~~
Java 언어는 모든 메소드를 `virtual`로 취급하고 있습니다.
~~~

`virtual` 키워드가 만들어진 배경에는 `vtable`을 사용하는데 드는 오버헤드때문이었습니다. 함수 주소값을 참조해서 실제 함수를 찾는 과정이 필요하기 때문에 성능 저하가 발생할 수 밖에 없었고, C++ 언어를 디자인하던 사람들은 이러한 이유 때문에 프로그래머에게 선택권을 주는 것이 더 낫다고 판단했습니다. (실제로 `virtual` 키워드를 사용하기 때문에 발생하는 오버헤드는 무시해도 될만큼 작습니다.)

<br>

# 소멸자의 virtual 필요성

모든 함수를 `virtual`로 선언하는 것에는 거부감이 있는 프로그래머들도 클래스의 소멸자만큼은 무조건 `virtual`로 선언해야 한다는 것에는 동의합니다.

만약 소멸자가 `virtual`이 아닌 경우에는 객체 소멸시 메모리 해제의 일부가 누락될 수 있는 상황이 발생할 수 있습니다. 

예를 들면 다음과 같은 상황입니다. 다음 상황에서 Parent 클래스의 소멸자가 `virtual`이 아닌 경우 문제가 발생합니다.

<pre class="prettyprint">
  Parent* p = new Child();
  delete p;
</pre>

위와 같은 경우는 `~Parent()`는 호출이 되지만, 소멸자가 `virtual`이 아니면 `~Child()`는 호출 되지 않습니다. 즉, Child 클래스의 소멸자에서 이루어지는 메모리 해제 등은 호출이 안되기 때문에 메모리 누수(Memory Leak) 등이 발생할 수 있습니다.