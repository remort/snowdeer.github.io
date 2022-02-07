---
layout: post
title: Git Submodule을 이용한 Python 라이브러리 패키지 관리(Docker 빌드 가능)
category: Python
tag: [Python]
---

# Git Submodule을 이용한 Python 라이브러리 패키지 관리

### Git Submodule을 이용한 프로젝트 추가

<pre class="prettyprint">
git submodule add https://github.com/snowdeer/python_sample_library libs/python_sample_library
</pre>

### Install Package

<pre class="prettyprint">
pip3 install libs/python_sample_library
</pre>

### Dockerfile 생성

<pre class="prettyprint">
FROM python:3.8
WORKDIR /app

# Install regular packages
COPY requirements.txt .
RUN pip install -r requirements.txt

# Install submodule packages
COPY libs/python_sample_library libs/python_sample_library
RUN pip3 install libs/python_sample_library

# copy source code
COPY ./ .

# command to run on container start
CMD [ "python", "./app.py"]
</pre>