---
layout: post
title: Python asyncio 및 Coroutine 예제
category: Python
tag: [Python, RabbitMQ]
---

# asyncio 및 Coroutine 예제

`asyncio`는 `select`와 동일한 방식으로 동작하는 이벤트 루프 모듈입니다. 
`asyncio`로 이벤트 루프를 생성하고, 어플리케이션은 특정 이벤트가 발생했을 때 호출할 함수를 등록합니다.
이러한 유형의 함수를 코루틴(Coroutine)이라고 합니다. 코루틴은 호출한 쪽에 제어를 되돌려 줄 수 있는
특별한 형태의 함수로 호출한 측에서 이벤트 루프를 계속 실행할 수 있게 합니다. 

코루틴은 `yield` 명령어를 이용해서 호출한 측에 제어권을 돌려주는 제너레이트와 동일하게 동작합니다.

## 간단한 예제

<pre class="prettyprint">
import asyncio
import time


async def hello() -> str:
    print('hello, snowdeer')

    for i in range(0, 5):
        print(f'hello({i})')
        time.sleep(1)

    return 'snowdeer'


hello_coroutine = hello()

print('* #1')
print(hello_coroutine)

print('* #2')
event_loop = asyncio.get_event_loop()
try:
    print("waiting event loop ...")
    result = event_loop.run_until_complete(hello_coroutine)
finally:
    event_loop.close()

print(f'result: {result}')
</pre>

결과는 다음과 같습니다.

```
* #1
<coroutine object hello at 0x100845c40>
* #2
waiting event loop ...
hello, snowdeer
hello(0)
hello(1)
hello(2)
hello(3)
hello(4)
result: snowdeer
```

## Coroutine에서 다른 Coroutine 호출하는 예제

<pre class="prettyprint">
import asyncio
import time


async def get_name() -> str:
    print('get_name() is called')

    for i in range(0, 3):
        print(f'get_name({i}) ...')
        time.sleep(1)

    return 'snowdeer'


async def hello() -> str:
    print('hello() is called')

    name = await get_name()

    for i in range(0, 3):
        print(f'hello, {name} ({i}) ...')
        time.sleep(1)

    return name


hello_coroutine = hello()

print('* #1')
print(hello_coroutine)

print('* #2')
event_loop = asyncio.get_event_loop()
try:
    print("waiting event loop ...")
    result = event_loop.run_until_complete(hello_coroutine)
finally:
    event_loop.close()

print(f'result: {result}')
</pre>

```
* #1
<coroutine object hello at 0x1054c1c40>
* #2
waiting event loop ...
hello() is called
get_name() is called
get_name(0) ...
get_name(1) ...
get_name(2) ...
hello, snowdeer (0) ...
hello, snowdeer (1) ...
hello, snowdeer (2) ...
result: snowdeer
```

`name = await get_name()` 코드에서 `await` 키워드는 `get_name()`이라는 코루틴을 이벤트 루프에 등록하고 
제어권을 이벤트 루프에 넘깁니다. 이벤트 루프는 제어권을 받아서 `get_name()` 코루틴을 실행한 다음 작업이 완료되면
이벤트 루프가 다시 기존의 `hello()` 코루틴 실행을 이어갑니다.

## await의 또 다른 예제

<pre class="prettyprint">
import asyncio
import time


async def loop1():
    print('loop1() is called')

    for i in range(0, 10):
        print(f'loop1({i}) ...')
        await time.sleep(1)


async def loop2():
    print('loop2() is called')

    for i in range(0, 10):
        print(f'loop2({i}) ...')
        await time.sleep(1)


event_loop = asyncio.get_event_loop()
try:
    print("waiting event loop ...")
    result = event_loop.run_until_complete(
        asyncio.gather(
            loop1(),
            loop2(),
        )
    )
finally:
    event_loop.close()
</pre>

와 같은 코드를 실행하면 다음과 같은 오류가 발생합니다.

```
waiting event loop ...
loop1() is called
loop1(0) ...
loop2() is called
loop2(0) ...
Traceback (most recent call last):
  File "/Users/snowdeer/Workspace/snowdeer/python_scalability/asyncio_example.py", line 24, in <module>
    result = event_loop.run_until_complete(
  File "/Applications/Xcode.app/Contents/Developer/Library/Frameworks/Python3.framework/Versions/3.8/lib/python3.8/asyncio/base_events.py", line 616, in run_until_complete
    return future.result()
  File "/Users/snowdeer/Workspace/snowdeer/python_scalability/asyncio_example.py", line 10, in loop1
    await time.sleep(1)
TypeError: object NoneType can't be used in 'await' expression
```

따라서 이런 경우는 `time.sleep()`가 아니라 `asyncio.sleep()`를 사용해야 합니다.
`asyncio.sleep()`는 `time.sleep()`와 달리 비동기 방식이기 때문에 지정된 시간까지 다른 일을 처리할 수 있습니다.

<pre class="prettyprint">
import asyncio
import time


async def loop1():
    print('loop1() is called')

    for i in range(0, 10):
        print(f'loop1({i}) ...')
        await asyncio.sleep(1)


async def loop2():
    print('loop2() is called')

    for i in range(0, 10):
        print(f'loop2({i}) ...')
        await asyncio.sleep(1)


event_loop = asyncio.get_event_loop()
try:
    print("waiting event loop ...")
    result = event_loop.run_until_complete(
        asyncio.gather(
            loop1(),
            loop2(),
        )
    )
finally:
    event_loop.close()
</pre>