---
layout: post
title: Thread Example
category: Python
tag: [Python]
---
# Python의 Thread

## GIL

대부분의 프로그램에서 Thread는 아주 많이 활용되고 있습니다. 하지만, Python의 Thread는 제약이 있습니다. GIL(Global Interpreter Lock)이라고 불리우는 전역 인터프리터 락이 존재합니다. GIL 때문에 여러 개의 코어(Core)를 갖고 있는 CPU에서 프로그램을 실행하더라도 단 하나의 코어만 활용하도록 되어 있습니다. 그 덕분에 Thread를 사용하게 되면 Thread를 사용하지 않았을 때보다 성능이 훨씬 더 저하되는 현상이 발생합니다.

한 동안 GIL을 제거하려는 논의가 있었지만, GIL은 여전히 존재하고 있습니다. GIL을 제거했을 때 얻는 장점과 추가로 생기는 단점의 큰 차이가 없어서 GIL을 유지하는 것으로 결정되었다고 합니다.

GIL은 CPU 기반의 작업에서만 아주 크게 영향을 미칩니다. I/O 작업(대부분의 시스템 콜(System Call)이나 네트워크 작업들)에서는 Thread를 사용할 경우 성능적인 장점이 있습니다. 시스템 콜을 하게 될 경우 그 순간 GIL은 해제되어 다른 작업을 할 수 있게 하고, 시스템 콜이 끝나면 다시 GIL을 획득하도록 되어 있어 Thread로 성능 향상을 가져올 수 있습니다.

그 외 CPU 기반의 다중 작업을 해야 하는 경우는 Thread 보다는 멀티프로세스(Multi Process)를 활용하는 것이 좋습니다.

Thread를 이용하는 방법은 여러가지가 있는데 그 중 `threading` 모듈을 이용해서 사용하는 것을 추천합니다. 저수준의 `_thread` 모듈과 고수준의 `threading` 모듈이 있는데 `threading` 모듈만 이용해도 대부분의 작업을 처리할 수 있으며 다뤄야 할 부분이 훨씬 적기 때문에 간단하고 안전하게 사용할 수 있기 때문입니다.

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