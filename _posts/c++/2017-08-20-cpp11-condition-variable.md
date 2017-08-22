---
layout: post
title: C++11 조건 변수(Condition Variable)
category: C++
tag: [C++, Thread]
---
# 조건 변수

조건 변수는 <condition_variable> 헤더 파일을 `include` 해야 사용할 수 있습니다.

조건 변수를 사용하면 특정 조건이 만족될 때까지 현재 Thread를 Blocking 할 수 있습니다.
이 때 특정 조건으로는 `notify` 이벤트를 주거나 타임 아웃 등이 될 수 있습니다.

조건 변수는 각 Thread를 Blocking 함으로 호출 순서를 조절하게 해서
결과적으로는 Thread 간 통신을 가능하게 해주는 효과를 가집니다.

<br>

# 조건 변수의 구성

조건 변수는 크게 `wait()`와 `notify_one()`이나 `notify_all()` 함수를 세트로 구성되어집니다.

`notify_one()` 함수는 해당 조건 변수를 기다리고 있는 Thread들 중 한 개의 Thread를 깨웁니다.
`notify_all()` 함수는 조건 변수를 기다리는 모든 Thread를 깨웁니다.

`wait()`를 호출하는 Thread는 먼저 락 객체를 점유하고 있는 상태여야 합니다. `wait()`를 호출하면
해당 락 객체의 `unlock()`이 호출되고 Thread가 Blocking 됩니다.

<br>

# 조건 변수의 활용

조건 변수를 가장 잘 활용할 수 있는 예제로 [메세지 큐(Message Queue)](/c++/2017/07/17/cpp11-message-queue/)를 들 수 있습니다.

다음과 같은 코드를 이용해서 Queue에 명령을 집어넣고 `notify_one()` 이벤트를 날립니다.

<pre class="prettyprint">
  void enqueue(T t) {
    unique_lock&lt;std::mutex&gt; lock(mMutex);
    mQueue.push(t);
    mCondition.notify_one();
  }
</pre>

<br>

또한 해당 Thread는 메세지를 처리하고, 다시 `wait()`로 Blocking 상태로 들어갑니다.

<pre class="prettyprint">
  T dequeue(void) {
   unique_lock&lt;std::mutex&gt; lock(mMutex);
    while((mIsLoop)&amp;&amp;(mQueue.empty())) {
      mCondition.wait(lock);
    }
    T val = mQueue.front();
    mQueue.pop();
    return val;
  }
</pre>