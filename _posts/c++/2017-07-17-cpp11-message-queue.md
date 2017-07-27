---
layout: post
title: Message 및 Message Queue
category: C++
tag: [C++11]
---

메세지(Message)와 메세지큐(MessageQueue)는 이벤트(Event)들을 한 곳에서 처리하거나 멀티 쓰레드(Multi-thread) 환경에서 쓰레드간 이벤트를 전달할 때 유용하게 사용됩니다.

C++로 구현된 메세지 및 메세지큐 예제입니다. 헤더 파일로만 이루어져 있어서 include 만으로 사용가능합니다. 또한 하단부에는 메세지와 메세지큐를 활용한 메세지 핸들러(Message Handler) 코드가 있습니다. 안드로이드의 핸들러(Handler)와 유사한 구조로 되어 있습니다.

<br>

# Message.h

<pre class="prettyprint">#ifndef SNOWDEER_MESSAGE_H
#define SNOWDEER_MESSAGE_H

class Message {
 public:
  Message(int _what) : what(_what), arg1(0), arg2(0) {}
  Message(int _what, int _arg1) : what(_what), arg1(_arg1), arg2(0) {}
  Message(int _what, int _arg1, int _arg2) : what(_what), arg1(_arg1), arg2(_arg2) {}
  virtual ~Message() {}

  int what;
  int arg1;
  int arg2;

  const static unsigned int MESSAGE_CLIENT_SOCKET_THREAD_CLOSED = 100;
};

#endif //SNOWDEER_MESSAGE_H
</pre>

<br>

# MessageQueue.h

<pre class="prettyprint">#ifndef SNOWDEER_MESSAGEQUEUE_H
#define SNOWDEER_MESSAGEQUEUE_H

#include &lt;queue&gt;
#include &lt;mutex&gt;
#include &lt;condition_variable&gt;

using namespace std;

template&lt;class T&gt;
class MessageQueue {
 public:
  MessageQueue(void) : mQueue(), mMutex(), mCondition() {
    mIsLoop = true;
  }

  ~MessageQueue(void) {
  }

  void clear() {
    std::lock_guard&lt;std::mutex&gt; lock(mMutex);
    std::queue&lt;T&gt; emptyQueue;
    std::swap(mQueue, emptyQueue);
  }

  void enqueue(T t) {
    std::lock_guard&lt;std::mutex&gt; lock(mMutex);
    mQueue.push(t);
    mCondition.notify_one();
  }

  void destory() {
    mIsLoop = false;
    mCondition.notify_one();
  }

  T dequeue(void) {
    std::unique_lock&lt;std::mutex&gt; lock(mMutex);
    while((mIsLoop)&amp;&amp;(mQueue.empty())) {
      mCondition.wait(lock);
    }
    T val = mQueue.front();
    mQueue.pop();
    return val;
  }

 private:
  std::queue&lt;T&gt; mQueue;
  mutable std::mutex mMutex;
  std::condition_variable mCondition;
  bool mIsLoop;
};

#endif //SNOWDEER_MESSAGEQUEUE_H
</pre>

<br>

# MessageHandler.h

<pre class="prettyprint">#ifndef SNOWDEER_MESSAGE_H
#define SNOWDEER_MESSAGE_H

class Message {
 public:
  Message(int _what) : what(_what), arg1(0), arg2(0) {}
  Message(int _what, int _arg1) : what(_what), arg1(_arg1), arg2(0) {}
  Message(int _what, int _arg1, int _arg2) : what(_what), arg1(_arg1), arg2(_arg2) {}
  virtual ~Message() {}

  int what;
  int arg1;
  int arg2;

  const static unsigned int MESSAGE_CLIENT_SOCKET_THREAD_CLOSED = 100;
};

#endif //SNOWDEER_MESSAGE_H</pre>


<br>

# MessageHandler.cc

<pre class="prettyprint">#include "MessageHandler.h"

MessageHandler::MessageHandler() {
  mIsRunning = false;
}

MessageHandler::~MessageHandler() {}

void MessageHandler::start() {
  mIsRunning = true;
  mLooper = std::thread(&amp;MessageHandler::loop, this);
}

void MessageHandler::stop() {
  mIsRunning = false;
  if(mLooper.joinable()) {
    mLooper.join();
  }
  clear();
  mMessageQueue.destory();
}

void MessageHandler::sendMessage(shared_ptr&lt;Message&gt; message) {
  mMessageQueue.enqueue(message);
}

void MessageHandler::clear() {
  mMessageQueue.clear();
}

void MessageHandler::loop() {
  while(mIsRunning) {
    shared_ptr&lt;Message&gt; message = mMessageQueue.dequeue();

    if(mIsRunning == false) {
      break;
    }

    switch(message-&gt;what) {
      case 1:
        break;

      case 2:
        break;
    }
  }
}</pre>
