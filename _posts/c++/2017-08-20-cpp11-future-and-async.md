---
layout: post
title: C++11 Future와 Async
category: C++
tag: [C++, Thread]
---
# Thread의 결과값 획득하는 방법

`std::future`와 `std::promise`를 이용하면 다른 Thread의 결과값을 쉽게 획득할 수 있습니다.
Thread에서 연산을 완료한 후 그 결과값은 `promise`에 저장합니다. 이후, `future`를 이용해서
그 값을 획득할 수 있습니다.

Thread 결과값은 다음과 같은 코드를 이용해서 획득가능합니다. Thread의 결과값을 받을 때까지
`get()` 부분은 Blocking 되어 대기합니다.

<pre class="prettyprint">
  future&lt;T&gt; fut = ...; 
  T res = fut.get();
</pre>

`promise`의 사용은 다음과 같이 할 수 있습니다.
<pre class="prettyprint">
  promise prom = ...;
  T val = ...;
  prom.set_value(val);
</pre>

<br>

# async 함수

`async` 함수를 사용하면 특정 함수 등을 Thread로 구동시키고 그 결과를 리턴받을 수 있습니다.
다음과 같은 예제 코드를 살펴보면, 

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;future&gt;

using namespace std;

int fun() {
  for (int i = 1; i <= 10; i++) {
    printf("fun[%d]\n", i);
  }

  return 200;
}

int main() {
  //auto fut = async(fun);
  auto fut = async(launch::async, fun);
  //auto fut = async(launch::deferred, fun);

  for (int i = 1; i <= 10; i++) {
    printf("main[%d]\t", i);
  }
  printf("\n");

  int result = fut.get();

  printf("result : %d\n", result);

  return 0;
}
</pre>

`async`를 통해 실행한 결과값을 `get()` 함수를 이용해서 돌려받을 수 있는 것을 확인할 수 있습니다.

위 예제에서 `async` 호출하는 부분을 3가지 예시로 들었는데, 만약

<pre class="prettyprint">
  auto fut = async(launch::async, fun);
</pre>

으로 수행하면, 함수 `fun()`은 즉시 실행이 되고 그 결과는 `future`에 저장이 됩니다.
(실행해보면 메인 Thread와 별도 Thread가 동시에 돌아가는 것을 확인할 수 있습니다.)

<br>

만약, `async` 부분을 다음과 같이 호출한 경우는

<pre class="prettyprint">
  auto fut = async(fun);    // 또는
  auto fut = async(launch::deferred, fun);
</pre>

`fun()` 함수는 바로 실행되는 것이 아니라 'int result = fut.get()' 코드가 실행될 때
`fun()` 함수가 실행되는 것을 확인할 수 있습니다.