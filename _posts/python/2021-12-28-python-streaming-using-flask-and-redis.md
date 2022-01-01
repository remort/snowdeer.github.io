---
layout: post
title: Python Streaming 서버 예제 (Flask와 Redis 사용)
category: Python
tag: [Python, RabbitMQ]
---

# Streaming 서버 예제 (Flask와 Redis 사용)

브라우저에서 [http://localhost:8001/message](http://localhost:8001/message)에 접속하면 
데이터가 스트리밍 되는 것을 확인할 수 있습니다.

## 예제 코드

<pre class="prettyprint">
import json

import flask
import redis
from flask import Flask

app = Flask(__name__)
app.debug = True


def stream_message(channel):
    r = redis.Redis()
    p = r.pubsub()
    p.subscribe(channel)
    for message in p.listen():
        if message['type'] == 'message':
            yield 'data: ' + json.dumps(message['data'].decode()) + '\n\n'


@app.route('/message', methods=['GET'])
def get_messages():
    return flask.Response(
        flask.stream_with_context(stream_message('snowdeer_channel')),
        mimetype='text/event-stream'
    )


if __name__ == '__main__':
    app.run(port=8001, use_reloader=False)
</pre>

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