---
layout: post
title: Condition Variable 활용한 Producer/Consumer 예제
category: Python
tag: [Python]
---
# Condition Variable

조건 변수(Condition Variable)는 내부에 Thread 대기 큐를 갖고 있습니다. `wait()` 메소드가 호출된 Thread는 이 대기 큐에 들어가게 되고 Sleep 상태가 되며, `notify()`나 `notifyAll()` 메소드에 의해 깨어나게 됩니다.

조건 변수를 활용한 생산자/소비자 패턴(Producer/Consumer Pattern) 예제는 다음과 같습니다.

<br>

## Producer/Consumber 패턴 예제

Producer는 물건(item)을 생산하여 `queue`에 차곡차곡 쌓아가며, Consumer는 `queue`에 쌓여있는 물건을 하나씩 가져갑니다. 단, `queue`에 쌓인 물건이 하나도 없을 경우에는 대기(wait)를 합니다.

<pre class="prettyprint">
import threading
import time

CONSUMER_COUNT = 10
PRODUCER_COUNT = CONSUMER_COUNT // 2

queue = []
cv = threading.Condition()

item_id = 0

class Consumer(threading.Thread):
    def __init__(self, id):
        threading.Thread.__init__(self)
        self.id = id

    def run(self):
        for i in range(5):
            cv.acquire()
            while len(queue) < 1:
                print('consumer({}) waiting...'.format(self.id))
                cv.wait()
            print('consumer({}) -> item({})'.format(self.id, queue.pop(0)))
            cv.release()
            time.sleep(0.5)


class Producer(threading.Thread):
    def run(self):
        global item_id
        for i in range(10):
            cv.acquire()
            item_id += 1
            queue.append(item_id)
            cv.notify()
            cv.release()
            time.sleep(0.7)


threads = []

for i in range(CONSUMER_COUNT):
    threads.append(Consumer(i))

for i in range(PRODUCER_COUNT):
    threads.append(Producer())

for th in threads:
    th.start()

for th in threads:
    th.join()

print('&lt;End&gt;')
</pre>

<br>

## 실행 결과

~~~
consumer(0) waiting...
consumer(1) waiting...
consumer(2) waiting...
consumer(3) waiting...
consumer(4) waiting...
consumer(5) waiting...
consumer(6) waiting...
consumer(7) waiting...
consumer(8) waiting...
consumer(9) waiting...
consumer(0) -> item(1)
consumer(1) -> item(2)
consumer(2) -> item(3)
consumer(3) -> item(4)
consumer(4) -> item(5)
consumer(4) waiting...
consumer(2) waiting...
consumer(3) waiting...
consumer(0) waiting...
consumer(1) waiting...
consumer(6) -> item(6)
consumer(7) -> item(7)
consumer(8) -> item(8)
consumer(5) -> item(9)
consumer(9) -> item(10)
consumer(9) waiting...
consumer(8) waiting...
consumer(5) waiting...
consumer(6) waiting...
consumer(7) waiting...
consumer(4) -> item(11)
consumer(2) -> item(12)
consumer(0) -> item(13)
consumer(3) -> item(14)
consumer(1) -> item(15)
consumer(1) waiting...
consumer(4) waiting...
consumer(0) waiting...
consumer(2) waiting...
consumer(3) waiting...
consumer(9) -> item(16)
consumer(8) -> item(17)
consumer(5) -> item(18)
consumer(6) -> item(19)
consumer(7) -> item(20)
consumer(9) waiting...
consumer(6) waiting...
consumer(7) waiting...
consumer(8) waiting...
consumer(5) waiting...
consumer(1) -> item(21)
consumer(4) -> item(22)
consumer(0) -> item(23)
consumer(3) -> item(24)
consumer(2) -> item(25)
consumer(1) waiting...
consumer(2) waiting...
consumer(3) waiting...
consumer(0) waiting...
consumer(4) waiting...
consumer(9) -> item(26)
consumer(6) -> item(27)
consumer(7) -> item(28)
consumer(8) -> item(29)
consumer(5) -> item(30)
consumer(9) waiting...
consumer(5) waiting...
consumer(8) waiting...
consumer(6) waiting...
consumer(7) waiting...
consumer(1) -> item(31)
consumer(3) -> item(32)
consumer(0) -> item(33)
consumer(2) -> item(34)
consumer(4) -> item(35)
consumer(1) waiting...
consumer(2) waiting...
consumer(3) waiting...
consumer(4) waiting...
consumer(0) waiting...
consumer(9) -> item(36)
consumer(5) -> item(37)
consumer(8) -> item(38)
consumer(6) -> item(39)
consumer(7) -> item(40)
consumer(9) waiting...
consumer(6) waiting...
consumer(7) waiting...
consumer(8) waiting...
consumer(5) waiting...
consumer(1) -> item(41)
consumer(2) -> item(42)
consumer(3) -> item(43)
consumer(4) -> item(44)
consumer(0) -> item(45)
consumer(9) -> item(46)
consumer(7) -> item(47)
consumer(6) -> item(48)
consumer(8) -> item(49)
consumer(5) -> item(50)
<End>
~~~

<br>

## 실제 Queue를 활용한 Producer/Consumer 패턴 예제

위의 예제에서는 `queue`라는 이름을 가진 리스트를 사용해서 구현했지만, 파이썬에서는 Multi-Threading 환경에서 사용할 수 있는 `queue` 모듈을 제공하고 있습니다.

`queue` 모듈을 사용한 예제는 아래와 같습니다.

<pre class="prettyprint">
import threading
import time
from queue import Queue

CONSUMER_COUNT = 10
PRODUCER_COUNT = CONSUMER_COUNT // 2

que = Queue(100)
item_id = 0

class Consumer(threading.Thread):
    def __init__(self, id):
        threading.Thread.__init__(self)
        self.id = id

    def run(self):
        for i in range(5):
            print('consumer({}) -> item({})'.format(self.id, que.get()))
            time.sleep(0)

class Producer(threading.Thread):
    def run(self):
        global item_id
        for i in range(10):
            item_id += 1
            que.put(item_id)
            time.sleep(0.7)

threads = []

for i in range(CONSUMER_COUNT):
    threads.append(Consumer(i))

for i in range(PRODUCER_COUNT):
    threads.append(Producer())

for th in threads:
    th.start()

for th in threads:
    th.join()

print('<&lt;End&gt;')
</pre>