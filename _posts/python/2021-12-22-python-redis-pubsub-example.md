---
layout: post
title: Redis Pub/Sub 예제
category: Python
tag: [Python, Redis]
---

# Redis Pub/Sub 예제

## Redis 설치

Redis는 Docker를 이용해서 설치해줍니다.

<pre class="prettyprint">
docker run --name redis -d -p 6379:6379 redis
</pre>

<br>

## subscriber.py

<pre class="prettyprint">
import redis

r = redis.Redis(host="localhost", port=6379, db=0)
s = r.pubsub()

s.subscribe("snowdeer_channel")

while True:
    print("waiting message...")
    res = s.get_message(timeout=5)
    if res is not None:
        print(f"res: {res}")
</pre>

<br>

## publisher.py

<pre class="prettyprint">
import redis
import datetime

r = redis.Redis(host="localhost", port=6379, db=0)

msg = f"[{datetime.datetime.now()}] hello, snowdeer +___+"
r.publish(channel="snowdeer_channel",
          message=msg)

print(f"Message is sent !!\n{msg}")
</pre>

## 여러 개의 Topic을 Subscription 하는 예제

<pre class="prettyprint">
import redis


def main():
    r = redis.Redis(host="localhost", port=6379, db=0)
    s = r.pubsub()

    s.subscribe('chat')
    s.subscribe('event')

    while True:
        print('waiting message...')
        res = s.get_message(timeout=5)
        if res is not None:
            if res['type'] == 'message':
                handle_message(res['channel'], res['data'])


def handle_message(topic: str, message: str):
    if topic == 'chat':
        handle_message_for_chat(message)
    elif topic == 'event':
        handle_message_for_event(message)


def handle_message_for_chat(message: str):
    print(f'chat message: {message}')


def handle_message_for_event(message: str):
    print(f'event message: {message}')


if __name__ == '__main__':
    main()
</pre>