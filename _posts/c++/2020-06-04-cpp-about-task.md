---
layout: post
title: Task
category: C++
tag: [C++]
---
# Task

C++ 에서는 Thread 외에도 비동기적으로 작업을 수행할 수 있는 태스크(Task를 지원합니다.
태스크는 `<future>` 헤더를 필요하며 Promise, Future 두 개의 컴포넌트로 구성됩니다.
이 두 컴포넌트는 서로 데이터 채널을 통해 연결됩니다.

<br>

# Thread와 Task 예제

<pre class="prettyprint">
#include &lt;iostream&gt;
#include &lt;future&gt;
#include &lt;thread&gt;

using namespace std;

int main() {
  cout << "Hello, World!" << endl;

  // Thread
  int count = 100;
  thread t([&] { count = count + 100; });
  t.join();
  cout << "count: " << count << endl;
  
  // Task
  auto f = async([&] { return count * 2; });
  cout << "count: " << f.get() << endl;

  return 0;
}
</pre>

위의 예제로 볼 수 있는 Thread와 Task의 차이는 다음과 같습니다.

기준 | Thread | Task
---|---|---
컴포넌트 | 생성자와 자식 Thread | Promise와 Future
통신 | 공유 변수 | 채널
동기화 | `join()`을 이용한 대기 | `get()`을 이용한 블록킹 호출
Exception 발생시 | Thread 및 부모 프로그램 종료됨 | Promise에서 Future로 Exception을 throw 함

Thread에서 데이터 통신을 위한 공유 변수는 Mutex 등으로 안전하게 보호해야 하지만, Task에서는
통신 채널이 이미 보호를 받고 있는 상태이기 때문에 Mutex 등을 사용할 필요가 없습니다.

Thread에서 예외가 발생하면, 해당 Thread는 종료가 되며 Thread 생성자 및 전체 프로그램도 종료가 됩니다.
하지만, Task에서는 Exception을 Future에게 발생시켜 예외 처리를 하도록 합니다.

<br>

## Future 및 Promise 예제

<pre class="prettyprint">
#include &lt;iostream&gt;
#include &lt;future&gt;
#include &lt;thread&gt;

using namespace std;

void add(promise&lt;int&gt; &&resultPromise, int a, int b) {
  resultPromise.set_value(a + b);
}

int main() {
  cout << "Hello, World!" << endl;

  promise&lt;int&gt; sumPromise;
  auto sumFuture = sumPromise.get_future();

  int a = 100;
  int b = 200;
  thread sumThread(add, move(sumPromise), a, b);

  cout << "a + b = " << sumFuture.get() << endl;

  sumThread.join();

  return 0;
}
</pre>