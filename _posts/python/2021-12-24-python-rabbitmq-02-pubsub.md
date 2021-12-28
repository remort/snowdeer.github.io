---
layout: post
title: Python에서 RabbitMq 사용하기 (2) - Publish/Subscribe
category: Python
tag: [Python, RabbitMQ]
---

# Publish/Subscribe

## Exchange

본격적인 `Exchange`에 대한 내용이 등장합니다. 
Publishe/Subscribe의 핵심은 하나의 메시지를 관심이 있는 여러 개의 Subscriber에게 전달하는 것입니다.

아래의 예시는 Logging 시스템이며, 하나의 프로그램에서 Log 메시지를 전송하면,
Disk에 Log을 기록하는 프로그램과 화면에 Log를 출력하는 프로그램이 각각 메시지를 수신합니다.

![image](/assets/python/003.png)

앞서 포스팅에서 Producer는 메시지를 절대 Queue에 직접 전송하지 않는다는 내용이 있습니다. 
Producer는 `Exchange`에게 메시지를 전송하며, `Exchange`에서 관련이 있는 `Queue`에 메시지를 
넣어 줍니다.

`Exchange`는 아래와 같이 선언 가능합니다.

<pre class="prettyprint">
channel.exchange_declare(exchange='logs',
                         exchange_type='fanout')
</pre>

이 때 `exchange_type`는 다음과 같은 값을 가질 수 있습니다.

- direct
- topic
- headers
- fanout

여기에서 `fanout`은 `Exchange`가 알고 있는 `Queue`들에게 메시지를 펼쳐서 전송하는 옵션으로
예시로 든 Logging 시스템에 적합한 타입입니다.

기존 포스팅에서 메시지를 전송하는 코드는 다음과 같았습니다.

<pre class="prettyprint">
channel.basic_publish(exchange='',
                      routing_key='hello',
                      body=message)
</pre>

`exchange` 값이 입력되지 않을 경우, default exchange를 사용하게 됩니다.
그리고, 메시지는 `routing_key` 값의 이름을 갖는 `Queue`에 전달됩니다.

## Temporary queues

Publish/Subscribe 방식으로 동작할 때 여러 개의 Subscriber를 동작시키려면 Queue가 여러 개 필요합니다.
이럴 때 여러 개의 Queue를 관리하는 것이 어려울 수 있습니다. 위의 예시에서 Disk Logging 프로그램과
Screen Logging 프로그램마다 각각의 Queue를 사용해야 하는데, 각 Queue의 이름을 `queue-disk-logging`, 
`queue-screen-logging`와 같이 정할 수도 있지만, 새로운 프로그램이 추가될 때마다 Queue의 이름을 계속 
짓는 것은 불편한 일이 될 수 있습니다. 

만약 Temporary queues 기능을 이용하면 Queue의 이름을 랜덤으로 지을 수 있습니다. 또한 `exclusive=True` 옵션을
이용해서 Subscriber가 종료되면 해당 Queue를 자동으로 없어지도록 할 수 있습니다.

Queue의 이름을 랜덤으로 짓는 방법은 다음과 같습니다.
<pre class="prettyprint">
result = channel.queue_declare(queue='', exclusive=True)
</pre>

이러면 랜덤으로 지어진 Queue의 이름은 `result.method.queue`으로 리턴되며, `amq.gen-JzTY20BRgKO-HjmUJj0wLg`와 같은
랜덤한 이름이 됩니다. 또한 `exclusive=True` 옵션에 의해 Subscriber가 종료되면 해당 Queue를 자동으로 지우도록 합니다.

## Binding

`Exchange`와 `Queue`를 연결하는 작업입니다. 아래와 같은 코드를 작성하면 `log`라는 이름의 `Exchange`와 랜덤으로 지어진 Queue를
연결합니다. 나중에 `log` 이름의 `Exchange`에 메시지가 들어오면 바인딩된 `Queue`들이 해당 메시지를 수신하게 됩니다.

<pre class="prettyprint">
channel.queue_bind(exchange='logs',
                   queue=result.method.queue)
</pre>

![image](/assets/python/004.png)

## publisher.py

아래는 Log 메시지를 Publish하는 예제입니다.

<pre class="prettyprint">
import pika
import sys

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='logs', exchange_type='fanout')

message = ' '.join(sys.argv[1:]) or "info: Hello World!"
channel.basic_publish(exchange='logs', routing_key='', body=message)
print(" [x] Sent %r" % message)
connection.close()
</pre>

기존 예제와 달라진, 눈여겨 볼 부분은 다음 코드입니다.

<pre class="prettyprint">
channel.basic_publish(exchange='logs', routing_key='', body=message)
</pre>
기존에는 `exchange`의 값이 비어있었고, `routing_key` 값에 `Queue`의 이름이 들어갔으나, 
지금 예제는 `exchange`의 값이 채워졌고, `routing_key` 값은 비어있습니다.

## subscriber.py

아래는 메시지를 Subscription하는 예제입니다.

<pre class="prettyprint">
import pika

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()

channel.exchange_declare(exchange='logs', exchange_type='fanout')

result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue

channel.queue_bind(exchange='logs', queue=queue_name)

print(' [*] Waiting for logs. To exit press CTRL+C')

def callback(ch, method, properties, body):
    print(" [x] %r" % body)

channel.basic_consume(
    queue=queue_name, on_message_callback=callback, auto_ack=True)

channel.start_consuming()
</pre>

여기에서 눈여겨 볼 부분은 아래의 코드입니다.

<pre class="prettyprint">
result = channel.queue_declare(queue='', exclusive=True)
queue_name = result.method.queue

channel.queue_bind(exchange='logs', queue=queue_name)
</pre>

랜덤한 이름의 Queue를 하나 생성하고, 그 Queue를 `logs`라는 이름의 `exchange`에 할당하는 코드입니다.