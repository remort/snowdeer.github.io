---
layout: post
title: venv 사용방법
category: Python
tag: [Python, venv]
---

# venv 사용방법

## venv 설치

Python 3.x 버전부터는 `virtualenv` 대신 `venv`를 사용하는 것이 더 간편합니다.
보통은 `venv`가 설치되어 있겠지만, 만약 설치되어 있지 않다면 다음 명령어로 설치할 수 있습니다. (Ubuntu 20.04 기준)

<pre class="prettyprint">
apt install python3.8-venv
</pre>

## venv 생성

<pre class="prettyprint">
python3 -m venv [env_name]

ex) python3 -m venv snowdeer_env
</pre>

`env_name`은 그냥 venv로 하면 편합니다. Pycharm 등의 IDE에서도 기본적으로는 이름을 venv로 설정합니다.

## venv 실행
위에서 명령어를 실행한 위치에 `venv` 디렉토리가 생성됩니다.
실행 방법은 해당 디렉토리내 `bin/activate` 파일을 실행하면 됩니다.

<pre class="prettyprint">
source snowdeer_env/bin/activate
</pre>