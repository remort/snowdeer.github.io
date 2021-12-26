---
layout: post
title: Celery 예제
category: Python
tag: [Python, Redis, Celery]
---

# Python에서 Celery 사용하기

`Celery`는 Task Queue입니다. RabbitMQ나 Redis 등을 백엔드로 사용하며 그 위에서 Wrapper로 추상화된 
인터페이스를 제공해줍니다.

## 설치

<pre class="prettyprint">
pip3 install redis
pip3 install celery
</pre>

<br>

## tasks.py

Task 실행 요청을 수신하는 서버측 코드입니다. 

<pre class="prettyprint">
import time

from celery import Celery

BROKER_URL = 'redis://localhost:6379/0'
BACKEND_URL = 'redis://localhost:6379/1'
app = Celery('tasks', broker=BROKER_URL, backend=BACKEND_URL)


@app.task(name="snowdeer_add")
def add(x, y):
    for i in range(1, 10):
        print("Calculating ...")
        time.sleep(0.5)

    return x + y
</pre>

실행은 일반적인 `python3`로 하는 것이 아니라 아래의 명령어로 실행합니다.

<pre class="prettyprint">
$ celery -A tasks worker --loglevel=info

 -------------- celery@Choongs-MacBook-Pro.local v5.2.1 (dawn-chorus)
--- ***** ----- 
-- ******* ---- macOS-12.0.1-arm64-arm-64bit 2021-12-25 19:59:29
- *** --- * --- 
- ** ---------- [config]
- ** ---------- .> app:         tasks:0x10273e430
- ** ---------- .> transport:   redis://localhost:6379/0
- ** ---------- .> results:     redis://localhost:6379/1
- *** --- * --- .> concurrency: 8 (prefork)
-- ******* ---- .> task events: OFF (enable -E to monitor tasks in this worker)
--- ***** ----- 
 -------------- [queues]
                .> celery           exchange=celery(direct) key=celery
                

[tasks]
  . snowdeer_add

[2021-12-25 19:59:29,936: INFO/MainProcess] Connected to redis://localhost:6379/0
[2021-12-25 19:59:29,945: INFO/MainProcess] mingle: searching for neighbors
[2021-12-25 19:59:30,982: INFO/MainProcess] mingle: all alone
[2021-12-25 19:59:31,035: INFO/MainProcess] celery@snowdeer-MacBook-Pro.local ready.
</pre>

<br>

## client.py

<pre class="prettyprint">
from tasks import add

f = add.apply_async(args=[2, 3, ])
result = f.get(timeout=10)

print(f"result: {result}")
</pre>

단순히 Task만 요청하고 싶을 때는 `add.apply_async(args=[2, 3, ])` 대신 `add.delay(2, 3)`과 같은 코드로 
대신할 수도 있습니다.