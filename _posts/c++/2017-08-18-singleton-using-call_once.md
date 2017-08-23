---
layout: post
title: call_once를 이용한 Singleton 패턴
category: C++
tag: [C++]
---
# Thread-Safe Singleton

기본적으로 Singleton 패턴은 Thread-Safe 하지 않습니다. 그래서 생성자에 `synchronized` 키워드나(Java의 경우) 'Mutex' 등을 이용해서 Thread-Safe 하도록 만들어주는 경우가 많습니다. 그게 아니면, 프로그램 실행 초반에 인스턴스를 생성하도록 하는 방법을 많이 씁니다.

C++11 부터 사용가능한 `call_once()`를 이용하면 좀 더 간편하게 Singleton 패턴을 사용할 수 있습니다.

`std::call_once()`는 `std::once_flag()`와 함께 사용하여 복수의 Thread 환경에서 특정 함수를 
단 한 번만 구동되도록 할 수 있습니다. 이러한 호출을 'Effective Call Once Invocation'라고 합니다.

<br>

# Singleton.h

<pre class="prettyprint">
#ifndef SNOWTHREAD_SINGLETON_H
#define SNOWTHREAD_SINGLETON_H

#include &lt;cstdio&gt;
#include &lt;mutex&gt;
#include &lt;memory&gt;

using namespace std;

class Singleton {
 public:
  static Singleton &getInstance() {
    call_once(Singleton::mOnceFlag, []() {
      printf("Singleton Instance is created...\n");
      mInstance.reset(new Singleton);
    });

    return *(mInstance.get());
  }

  void log() {
    printf("hello\n");
  }


 private:
  static unique_ptr&lt;Singleton&gt; mInstance;
  static once_flag mOnceFlag;

  Singleton() = default;
  Singleton(const Singleton &) = delete;
  Singleton &operator=(const Singleton &) = delete;
};

unique_ptr&lt;Singleton&gt; Singleton::mInstance;
once_flag Singleton::mOnceFlag;

#endif //SNOWTHREAD_SINGLETON_H
</pre>

<br>

# main.cpp

<pre class="prettyprint">
#include "Singleton.h"

int main() {
  Singleton::getInstance().log();
  Singleton::getInstance().log();

  return 0;
}
</pre>

<br>

# 실행 결과

<pre class="prettyprint">
Singleton Instance is created...
hello
hello
</pre>