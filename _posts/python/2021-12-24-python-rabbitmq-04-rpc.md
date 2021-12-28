---
layout: post
title: Python에서 RabbitMq 사용하기 (4) - RPC
category: Python
tag: [Python, RabbitMQ]
---

# RPC

지금까지의 예제는 단방향으로 메시지를 전송하고 끝나는 예제들이었습니다. 하지만, 함수처럼 결과를 기다려야 할 때는 RPC 패턴을 사용합니다.

개인적으로는 RabbitMQ의 RPC 보다는 [Celery](https://docs.celeryproject.org/en/stable/)의 Backend Result가 더 
좋은 것 같습니다. RabbitMQ나 Redis는 Message Queue로 사용하고, 여기에 Task Queue인 Celery를 같이 사용하는 편이 더 사용성이 좋은 것 같습니다.

암튼 RabbitMQ의 RPC를 살펴보면, RabbitMQ는 `callback_queue`를 이용해서 RPC 기능을 지원합니다. Client에서 Request 메시지를 보내면
Server에서는 Response 메시지를 Client에게 전송합니다. Client에서 Response 메시지를 받기 위해서는 Request 메시지를 전송할 때, 
아래의 예제와 같이 `callback queue`의 주소를 같이 보내야 합니다.

<pre class="prettyprint">
result = channel.queue_declare(queue='', exclusive=True)
callback_queue = result.method.queue

channel.basic_publish(exchange='',
                      routing_key='rpc_queue',
                      properties=pika.BasicProperties(
                            reply_to = callback_queue,
                            ),
                      body=request)
</pre>

위 방법으로 RPC를 구현하면 Request를 보낼 때마다 `callback_queue`를 매번 생성하기 때문에 비효율적인 면이 있습니다.
따라서 Client에서는 하나의 `callback_queue`를 만들어서 다양한 메시지의 응답을 수신하는 것이 더 유리합니다.
그러기 위해서는 `properties`의 `correlation_id`라는 속성을 이용하면 됩니다. `correlation_id`는 Request마다
고유의 값을 가집니다. 그래서 하나의 `callback_queue`를 사용하더라도 Response가 어떤 Request에 대한 것인지 알 수 있습니다.

![image](/assets/python/009.png)

## RPC 과정

따라서 RPC를 구현하는 과정은 다음과 같습니다.

* Client 시작할 때, `exclusive=True`인 랜덤 이름의 Queue를 하나 생성
* RPC Request를 보낼 때, `reply_to`, `correlation_id` 2개의 항목의 값을 채워서 전송. `reply_to`는 `callback_queue`의 이름, `correlation_id`는 각 Request 마다 고유한 값으로 설정
* Request는 `rpc_queue`에 전송됨
* 서버는 메시지 수신 후 작업을 수행함. 그리고 결과는 `reply_to` 필드 이름의 Queue로 전송
* Client는 `callback_queue`에 메시지가 도착하는 것을 기다리며, `correlation_id` 값을 확인 후 메시지 수신

## rpc_server.py

<pre class="prettyprint">
import pika

connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))

channel = connection.channel()

channel.queue_declare(queue='rpc_queue')

def fib(n):
    if n == 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fib(n - 1) + fib(n - 2)

def on_request(ch, method, props, body):
    n = int(body)

    print(" [.] fib(%s)" % n)
    response = fib(n)

    ch.basic_publish(exchange='',
                     routing_key=props.reply_to,
                     properties=pika.BasicProperties(correlation_id = \
                                                         props.correlation_id),
                     body=str(response))
    ch.basic_ack(delivery_tag=method.delivery_tag)

channel.basic_qos(prefetch_count=1)
channel.basic_consume(queue='rpc_queue', on_message_callback=on_request)

print(" [x] Awaiting RPC requests")
channel.start_consuming()
</pre>

## rpc_client.py

<pre class="prettyprint">
import pika
import uuid

class FibonacciRpcClient(object):

    def __init__(self):
        self.connection = pika.BlockingConnection(
            pika.ConnectionParameters(host='localhost'))

        self.channel = self.connection.channel()

        result = self.channel.queue_declare(queue='', exclusive=True)
        self.callback_queue = result.method.queue

        self.channel.basic_consume(
            queue=self.callback_queue,
            on_message_callback=self.on_response,
            auto_ack=True)

    def on_response(self, ch, method, props, body):
        if self.corr_id == props.correlation_id:
            self.response = body

    def call(self, n):
        self.response = None
        self.corr_id = str(uuid.uuid4())
        self.channel.basic_publish(
            exchange='',
            routing_key='rpc_queue',
            properties=pika.BasicProperties(
                reply_to=self.callback_queue,
                correlation_id=self.corr_id,
            ),
            body=str(n))
        while self.response is None:
            self.connection.process_data_events()
        return int(self.response)


fibonacci_rpc = FibonacciRpcClient()

print(" [x] Requesting fib(30)")
response = fibonacci_rpc.call(30)
print(" [.] Got %r" % response)
</pre>