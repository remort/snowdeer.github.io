---
layout: post
title: C++11 Thread 생성 방법들
category: C++
tag: [C++, Thread]
---
## 함수 포인터를 이용하는 방법

함수 포인터를 이용하는 예제 코드는 다음과 같습니다.

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;thread&gt;

using namespace std;

void counter(int id, int length) {
  for(int i=1; i<=length; i++) {
    printf("counter[%d] : %d\n", id, i);
  }
}

int main() {
  thread t1(counter, 1, 5);
  thread t2(counter, 2, 7);
  t1.join();
  t2.join();

  return 0;
}
</pre>

위에서 `join()`은 각 Thread가 작업 완료될 때까지 Blocking되어 있도록 하는
명령어입니다. Blocking은 일반적으로 자원의 낭비를 가져오기 때문에 
실제 프로그램에서는 `join()`의 사용을 최대한 피하는 것이 좋습니다. 대신 
Thread에 메세지(Message)를 처리하는 루틴을 만들고, Thread에 메세지를 보내어서
작업을 수행하는 방식이 좀 더 바람직합니다.

<br>

## 함수 객체를 이용하는 방법

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;thread&gt;

using namespace std;

class Counter {
 public:
  Counter(int id, int length) {
    mId = id;
    mLength = length;
  }

  void operator()() const {
    for (int i = 1; i <= mLength; i++) {
      printf("counter[%d] : %d\n", mId, i);
    }
  }

 private:
  int mId;
  int mLength;
};

int main() {
  // #1
  thread t1{Counter(1, 5)};

  // #2
  Counter c2(2, 7);
  thread t2(c2);

  // #3
  thread t3(Counter(3, 8));

  t1.join();
  t2.join();
  t3.join();

  return 0;
}
</pre>

위의 예제에서 3가지 방식이 있었는데 3번째 방식은 특수한 경우(예를 들어 인자로 들어가는 클래스의 생성자에 파라메터가 없는 경우)에 컴파일 에러가 뜰 수 있기 때문에 가급적 첫 번째 방식을 사용하는 편이 더 낫습니다.

<br>

## 람다 표현식을 이용하는 방법

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;thread&gt;

using namespace std;

int main() {
  thread t1([](int id, int length) {
    for (int i = 1; i <= length; i++) {
      printf("counter[%d] : %d\n", id, i);
    }
  }, 1, 7);

  t1.join();

  return 0;
}
</pre>

<br>

## 클래스 메소드를 이용하는 방법

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;thread&gt;

using namespace std;

class Counter {
 public:
  Counter(int id, int length) {
    mId = id;
    mLength = length;
  }

  void loop() const {
    for (int i = 1; i <= mLength; i++) {
      printf("counter[%d] : %d\n", mId, i);
    }
  }

 private:
  int mId;
  int mLength;
};

int main() {
  Counter c1(1, 7);

  thread t1{&Counter::loop, &c1};

  t1.join();

  return 0;
}
</pre>

이 방법은 특정 인스턴스의 메소드를 별도 Thread로 실행시킬 수 있는 장점이 있습니다.