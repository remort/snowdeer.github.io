---
layout: post
title: Python에서 RabbitMq 사용하기 (1) - RabbitMQ 기본 동작
category: Python
tag: [Python, RabbitMQ]
---

# RabbitMQ 기본 동작

RabbitMQ는 프로그램간 메시지를 쉽게 주고 받을 수 있도록 하는 Message Queue입니다.
기본적으로 Producer에서 생성한 Message를 Queue를 이용해서 Consumer에게 전달하는 
Producer-Consumer 패턴으로 되어 있으며, Producer와 Consumer간 느슨한 결합을 할 수 있게 해줍니다.

기본적인 동작을 대략적인 그림으로 표현하면 다음과 같습니다.

![image](/assets/python/001.png)

하지만, 실제 동작은 위 그림과는 조금 다릅니다. RabbitMQ에서는 Producer에서 `Queue`로 직접 Message를 보내는 경우는
없습니다. Message는 `Exchange`라는 요소를 거쳐서 `Queue`에 전달되지만, `Exchange`에 대한 내용은 후반에 다룰 
예정이기 때문에 위 그림에서는 생략되어 있습니다.

## sender.py

`sender`는 정의한 `Queue`에 메시지를 전송합니다. 실제로는 직접적으로 `Queue`에 메시지를 넣는 것이 아니라
`Exchange`에 메시지를 입력하지만, 아래 예제에서는 `Exchange`를 default 값으로 지정했습니다.

메시지를 전송하는 코드는 다음과 같습니다.

<pre class="prettyprint">
channel.basic_publish(exchange='', routing_key=QUEUE_NAME, body=msg)
</pre>

전체 코드는 다음과 같습니다.

<pre class="prettyprint">
from datetime import datetime
import pika

HOST_NAME = "localhost"
QUEUE_NAME = "snowdeer_queue"


def main():
    connection = pika.BlockingConnection(pika.ConnectionParameters(host=HOST_NAME))
    channel = connection.channel()

    channel.queue_declare(queue=QUEUE_NAME)

    msg = f"[{datetime.now()}] hello, snowdeer !!"
    channel.basic_publish(exchange='', routing_key=QUEUE_NAME, body=msg)
    print(f"Sent message.\n{msg}")
    connection.close()


if __name__ == '__main__':
    main()
</pre>

## receiver.py

메시지를 수신하는 예제입니다. `callback` 함수를 통해 메시지를 수신하며, 메시지를 대기하는 부분은
다음과 같습니다.

<pre class="prettyprint">
channel.start_consuming()
</pre>

전체 코드입니다.

<pre class="prettyprint">
import pika

HOST_NAME = "localhost"
QUEUE_NAME = "snowdeer_queue"


def main():
    connection = pika.BlockingConnection(pika.ConnectionParameters(host=HOST_NAME))
    channel = connection.channel()

    channel.queue_declare(queue=QUEUE_NAME)

    def callback(ch, method, properties, body):
        print("Message is Arrived %r" % body)

    channel.basic_consume(queue=QUEUE_NAME,
                          on_message_callback=callback,
                          auto_ack=True)

    try:
        print("Waiting for messages.")
        channel.start_consuming()
    except KeyboardInterrupt:
        print('Ctrl+C is Pressed.')


if __name__ == '__main__':
    main()
</pre>

## Message의 생명 주기

만약 RabbitMQ의 `Queue`에 메시지가 입력되었는데, Consumer가 메시지를 가져가기 전에 RabbitMQ가 
재실행되는 상황이 발생하게 된다면, Queue의 메시지가 삭제됩니다. 
중요한 메시지라면 메시지의 내구성이 매우 중요하며, 아래 명령어를 이용해서 Queue에 입력된 메시지를
Disk에 저장하여 메시지의 내구성을 높일 수 있습니다.

<pre class="prettyprint">
channel.queue_declare(queue='task_queue', durable=True)
</pre>

위 명령어는 기존에 존재하지 않던 `Queue`를 새로 정의할 때만 적용되며, 이미 정의되어있는 `Queue`에 대해서는
동작하지 않습니다.

그리고 메시지를 전송할 때도 다음과 같이 내구성을 적용할 수 있습니다.

<pre class="prettyprint">
channel.basic_publish(exchange='',
                      routing_key="task_queue",
                      body=message,
                      properties=pika.BasicProperties(
                         delivery_mode = pika.spec.PERSISTENT_DELIVERY_MODE
                      ))
</pre>

## 하나의 Queue에 복수 Receiver 연결

만약 하나의 `Queue`에 여러 개의 Receiver를 연결하면 어떻게 될까요?
각 Receiver들이 차례대로 하나씩 메시지를 가져갑니다. 

![image](/assets/python/002.png)

이 빈도는 아래 명령어를 이용해서 조정할 수 있습니다.

<pre class="prettyprint">
channel.basic_qos(prefetch_count=1)
</pre>