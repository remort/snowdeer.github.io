---
layout: post
title: Python에서 RabbitMq 사용하기 (3) - Routing
category: Python
tag: [Python, RabbitMQ]
---

# Routing

Routing 기능은 제 기준에서 그리 많이 사용되진 않을 거 같아서 간단히만 공부합니다.
더 자세한 내용은 [여기](https://www.rabbitmq.com/tutorials/tutorial-four-python.html)에서 확인할 수 있습니다.

이전 포스팅에서 설명했던 Publish/Subscribe 기능은 단순히 메시지를 모든 Subscriber에게 전송하는 기능입니다. 
여기에 약간의 옵션을 추가해서, 특정 Subscriber는 특별한 메시지만 받도록 할 수 있습니다.

예를 들어, 앞선 예제의 Logging 시스템에서 Disk Logging 프로그램, Screen Logging 프로그램이 있을 때
Screen Logging 프로그램은 모든 메시지를 수신하고, Disk Logging 프로그램은 Disk의 용량 절약을 목적으로
Critical Error Message만 수신하고 싶은 경우에 Routing 기능을 사용할 수 있습니다.

이 때, `routing_key` 옵션을 이용해서 바인딩 키를 설정할 수 있습니다.

<pre class="prettyprint">
channel.queue_bind(exchange=exchange_name,
                   queue=queue_name,
                   routing_key='black')
</pre>

바인딩 키는 `Exchange`의 타입에 영향을 받습니다. 타입이 `fanout`인 경우는 바인딩 키가 무시됩니다. 

따라서, 아래 예제는 `Exchange`를 생성할 때, 타입을 `direct`로 설정했습니다.

![image](/assets/python/005.png)

위 그림에서 각 메시지는 바인딩 키(`routing key`)가 등록된 Queueㄹ 전송이 됩니다.
물론, 아래 그림과 같이 여러 개의 Queue에 동일한 바인딩 키를 바인딩해도 상관없습니다.

![image](/assets/python/006.png)

## 예제 코드

다음은 `Exchange`를 `direct` 타입으로 생성하는 코드입니다.

<pre class="prettyprint">
channel.exchange_declare(exchange='direct_logs',
                         exchange_type='direct')                 
</pre>

그리고 Publish 할 때 아래와 같이 `routing_key`에 값을 추가해서 메시지를 전송하면 됩니다.

<pre class="prettyprint">
channel.basic_publish(exchange='direct_logs',
                      routing_key=severity,
                      body=message)
</pre>

## Logging 시스템 예제 코드

![image](/assets/python/007.png)

### log_emitter.py

<pre class="prettyprint">
import pika
import sys

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='direct_logs', exchange_type='direct')

severity = sys.argv[1] if len(sys.argv) > 1 else 'info'
message = ' '.join(sys.argv[2:]) or 'Hello World!'
channel.basic_publish(
    exchange='direct_logs', routing_key=severity, body=message)
print(" [x] Sent %r:%r" % (severity, message))
connection.close()
</pre>

### log_receiver.py

<pre class="prettyprint">
import pika
import sys

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='direct_logs', exchange_type='direct')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue

severities = sys.argv[1:]
if not severities:
    sys.stderr.write("Usage: %s [info] [warning] [error]\n" % sys.argv[0])
    sys.exit(1)

for severity in severities:
    channel.queue_bind(
        exchange='direct_logs', queue=queue_name, routing_key=severity)

print(' [*] Waiting for logs. To exit press CTRL+C')


def callback(ch, method, properties, body):
    print(" [x] %r:%r" % (method.routing_key, body))


channel.basic_consume(
    queue=queue_name, on_message_callback=callback, auto_ack=True)

channel.start_consuming()
</pre>

## Topic

그 외 아래 그림처럼 다양하고 자유로운 형식의 `routing_key`를 활용하기 위해서는 `exchange_type`을 `topic`으로 할 수 있습니다.

![image](/assets/python/008.png)

`routing_key`를 `<celerity>.<colour>.<species>`와 같은 형태로 하고, 각 Queue의 바인딩 키를 
`*.orange.*`,  `*.*.rabbit`, `lazy.#` 등으로 설정할 수 있습니다.