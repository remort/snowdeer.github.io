---
layout: post
title: Python Socket으로 HTTP Request 보내는 방법
category: Python
tag: [Python, RabbitMQ]
---

# Socket으로 HTTP Request 보내는 예제 코드

`HTTP 1.1` 부터는 Header 정보에 `Host` 정보를 넣어줘야 하며, 마지막 부분에 2개의 `\r\n`이 필요합니다.

다음 2가지 예제는  모두 같은 코드입니다.

<pre class="prettyprint">
import socket

s = socket.create_connection(('httpbin.org', 80))
s.sendall(b'GET / HTTP/1.1\r\nHost: httpbin.org\r\n\r\n')

buf = s.recv(1024)
print(buf)
</pre>

<pre class="prettyprint">
import socket

s = socket.create_connection(('httpbin.org', 80))
s.sendall(b'''GET / HTTP/1.1
Host: httpbin.org

''')

buf = s.recv(1024)
print(buf)
</pre>