---
layout: post
title: C++11 기반 클래스 멤버 함수 콜백(Callback)
category: C++
tag: [C++]
---
# 함수 콜백 예제

## Child.h

<pre class="prettyprint">
#ifndef FUNCTIONCALLBACK_CHILD_H
#define FUNCTIONCALLBACK_CHILD_H

#include &lt;functional&gt;

using namespace std;

class Child {
 public:
  Child() {}
  virtual ~Child() {}

 private:
  typedef function<void()> func1;
  typedef function<void(int, int)> func2;

  func1 handler1;
  func2 handler2;

 public:
  void setHandler1(func1 func) {
    handler1 = move(func);
  }

  void setHandler2(func2 func) {
    handler2 = move(func);
  }

  void exec() {
    handler1();
    handler2(100, 200);
  }
};

#endif //FUNCTIONCALLBACK_CHILD_H
</pre>

<br>

## parent.h

<pre class="prettyprint">
#ifndef FUNCTIONCALLBACK_PARENT_H
#define FUNCTIONCALLBACK_PARENT_H

#include "Child.h"

#include &lt;iostream&gt;
#include &lt;functional&gt;

using namespace std;

class Parent {
 public:
  Parent() {}
  virtual ~Parent() {}

  void hello() {
    cout << "Hello" << endl;
  }

  void sum(int a, int b) {
    cout << a << " + " << b << " = " + (a + b) << endl;
  }

  void test() {
    Child child;

    child.setHandler1(bind(&Parent::hello, this));
    child.setHandler2(bind(&Parent::sum, this, placeholders::_1, placeholders::_2));
    child.exec();
  }
};

#endif //FUNCTIONCALLBACK_PARENT_H
</pre>

<br>

## main.cc

<pre class="prettyprint">
#include "Parent.h"

int main() {

  Parent obj;
  obj.test();

  return 0;
}
</pre>