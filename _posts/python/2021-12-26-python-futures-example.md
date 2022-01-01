---
layout: post
title: Python futures 예제
category: Python
tag: [Python, RabbitMQ]
---

# Futures

파이썬 3.2에 도입된 `concurrent.futures` 모듈이며, 나중에 파이썬 2에도 적용되었습니다.
파이썬은 `concurrent.futures.ThreadPoolExecutor`와 `concurrent.futures.Process.PoolExecutor`의 2종류의
`Executor`를 지원합니다. 각각 Thread 기반과 Process 기반입니다.

## 간단한 예제

<pre class="prettyprint">
import time
from concurrent import futures


def loop(name, step):
    __sum = 0
    for i in range(0, 10):
        __sum = __sum + step
        print(f'[{name}] is looping({i}) ...')
        time.sleep(0.1)

    return __sum


if __name__ == '__main__':
    print(f'calculating ...')
    with futures.ThreadPoolExecutor(max_workers=3) as executor:
        f1 = executor.submit(loop, 'apple', 1)
        f2 = executor.submit(loop, 'bread', 2)
        f3 = executor.submit(loop, 'carrot', 3)
        f4 = executor.submit(loop, 'david', 4)
        f5 = executor.submit(loop, 'egg', 5)

    print(f'<result>')
    print(f'result #1 : {f1.result()}')
    print(f'result #2 : {f2.result()}')
    print(f'result #3 : {f3.result()}')
    print(f'result #4 : {f4.result()}')
    print(f'result #5 : {f5.result()}')
</pre>

