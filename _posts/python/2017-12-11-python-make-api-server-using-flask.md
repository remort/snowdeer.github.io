---
layout: post
title: Flask 모듈 사용하기
category: Python
tag: [Python]
---
# Flask 모듈

Flask 모듈은 웹 어플리케이션을 쉽게 만들 수 있는 여러 가지 기능들을 제공하는 마이크로 웹 프레임워크입니다.

<br>

## Flask 설치

Flask는 `pip` 명령어를 이용해서 쉽게 설치할 수 있습니다.

~~~
$ pip install Flask
~~~

<br>

## Hello, World 서버 만들기

<pre class="prettyprint">
from flask import Flask

app = Flask(__name__)

app.debug = True


@app.route('/')
def hello():
    return 'Hello, World'


if __name__ == '__main__':
    app.run()
</pre>

<br>

## URL 경로를 이용해서 함수 실행하기

아래와 같은 코드를 이용해서 URL 경로를 파라메터로 받을 수 있습니다.

<pre class="prettyprint">
@app.route('/hello/&lt;name&gt;')
def hello_to(name):
    return 'Hello, {}'.format(name)
</pre>

<br>

## URL 쿼리 가져오기

'http://127.0.0.1:5000/query?x=100&y=200' 와 같은 URL 속에 같이 포함되어 있는 쿼리값을 가져오는 코드는 다음과 같습니다.

<pre class="prettyprint">
from flask import Flask
from flask import request

app = Flask(__name__)

app.debug = True


@app.route('/')
def hello():
    return 'Hello'


@app.route('/hello/&lt;name&gt;')
def hello_to(name):
    return 'Hello, {}'.format(name)


@app.route('/query')
def get_query_params():
    x = request.args.get('x')
    y = request.args.get('y')

    return 'x = {}, y = {}'.format(x, y)


if __name__ == '__main__':
    app.run()
</pre>
