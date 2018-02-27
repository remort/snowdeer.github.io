---
layout: post
title: Dockerfile for Python
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Dockerfile for Python

Python 용 어플리케이션을 위한 Dockerfile 템플릿입니다.

<br>

## Dockerfile

Python 3.x 버전의 경우는

<pre class="prettyprint">
FROM python:3

WORKDIR /usr/src/app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD [ "python", "./your-daemon-or-script.py" ]
</pre>

와 같으며,

<br>

Python 2.x 버전에서는 다음과 같이 작성합니다.

<pre class="prettyprint">
FROM python:2

WORKDIR /usr/src/app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD [ "python", "./your-daemon-or-script.py" ]
</pre>

<br>

## requirements.txt

<pre class="prettyprint">
flask
</pre>

<br>