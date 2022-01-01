---
layout: post
title: Python Blocking Socket 및 Non-Blocking Socket 예제
category: Python
tag: [Python, RabbitMQ]
---

# Python Blocking Socket 및 Non-Blocking Socket 예제

## Blocking Socket 예제

다음 코드는 [httpbin.org](httpbin.org)의 `/delay/3` 서브도메인에 접속해서 3초 딜레이 이후 응답을 받는 코드입니다.
`s.recv` 코드에서 3초동안 Blocking 되서 동작합니다.

<pre class="prettyprint">
import socket

s = socket.create_connection(('httpbin.org', 80))
s.sendall(b'GET /delay/3 HTTP/1.1\r\nHost: httpbin.org\r\n\r\n')

buf = s.recv(1024)
print(buf)
</pre>

## Non-Blocking Socket 예제

다음은 Non-Blocking으로 동작하기 때문에 중간에 `while` 문으로 응답이 완료될 때까지 대기합니다.

<pre class="prettyprint">
import select
import socket

s = socket.create_connection(('httpbin.org', 80))
s.setblocking(False)

s.sendall(b'GET /delay/3 HTTP/1.1\r\nHost: httpbin.org\r\n\r\n')

while True:
    ready_to_read, ready_to_write, in_error = select.select([s], [], [])
    if s in ready_to_read:
        break

buf = s.recv(1024)
print(buf)
</pre>

## select

여기에서 사용한 `select`는 여러 개의 이벤트 소스를 결합하여 쉽게 모니터링할 수 있는 기술이며, 
오래된 기술입니다. 그래서 최상의 성능을 보여주지는 못하며 리눅스의 `epoll`이나 FreeBSD의 `kqueue`와 같이
운영체제마다 다른 대안과 최적화를 구현해서 사용하고 있습니다.

파이썬에서는 다음 포스팅에서 다룰 `asyncio`라는 추상화 계층을 이용해서 사용하는 것이 더 좋습니다.