---
layout: post
title: Thread Example
category: Python
tag: [Python]
---
# Python의 Thread

Python의 Thread는 제약이 있습니다. 내부적으로 전역 인터프리터 락(GIL, Global Interpreter Lock)을 사용하는데, 이 덕분에 시스템 하나에서는 하나의 Thread만 동작되도록 제한하고 있습니다. 즉, 코어(Core)가 여러 개인 시스템에서도 하나의 코어만 사용하기 때문에 제약이 있습니다. 그 이후 GIL을 제거하려는 논의가 있었으나 GIL을 제거했을 때 얻는 장점과 추가로 생기는 오버헤드가 큰 차이가 없어서 GIL은 그대로 유지하는 것으로 결정이 되었습니다.

하지만, 현재는 `threading` 모듈을 이용해서 여러 개의 코어를 사용하는 Multi-Threading 프로그램을 개발할 수 있게 되었습니다. Multi-Threading을 위해서는 저수준의 `_thread` 모듈과 고수준의 `threading` 모듈이 있는데 `threading` 모듈만 이용해도 대부분의 작업을 처리할 수 있습니다.

<br>

## 특정 함수를 Multi-Threading으로 실행

<pre class="prettyprint">
import threading
import time


def loop(id):
    print('Thread({}) is starting...'.format(id))

    for i in range(10):
        print('Thread({}) - {}'.format(id, i))
        time.sleep(0)

    print('Thread({}) is finished...'.format(id))


threads = []

if __name__ == '__main__':
    for i in range(5):
        th = threading.Thread(target=loop, args=(i,))
        threads.append(th)
        th.start()

    for th in threads:
        th.join()

    print('&lt;End&gt;')
</pre>

<br>

## 특정 클래스를 Multi-Threading으로 실행

<pre class="prettyprint">
import threading
import time

class MyThread(threading.Thread):
    def run(self):
        print('Thread({}) is starting...'.format(self.getName()))

        for i in range(10):
            print('Thread({}) - {}'.format(self.getName(), i))
            time.sleep(0)

        print('Thread({}) is finished...'.format(self.getName()))

threads = []

if __name__ == '__main__':
    for i in range(5):
        th = MyThread()
        threads.append(th)
        th.start()

    for th in threads:
        th.join()

    print('&lt;End&gt;')
</pre>

<br>

## Thread Lock

<pre class="prettyprint">
import threading
import time

count = 0

class MyThread(threading.Thread):
    def run(self):
        global count

        for i in range(10):
            lock.acquire()
            count += 1
            lock.release()

threads = []
lock = threading.Lock()

if __name__ == '__main__':
    for i in range(5):
        th = MyThread()
        threads.append(th)
        th.start()

    for th in threads:
        th.join()

    print('Total Count: {}'.format(count))
</pre>

만약 `acquire()` 메소드를 여러 번 호출하려면 `Lock` 대신 `RLock` 클래스를 활용하면 됩니다. 물론, 해제할 때는 `acquire()` 메소드 호출 횟수만큼 `release()`를 호출해야합니다.