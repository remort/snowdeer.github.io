---
layout: post
title: 라즈베리파이에 Python 설치
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스, 파이썬]
---
# 라즈베리파이에 Python 3.x 버전 설치하기

라즈베리파이에는 기본적으로 Python 2.x 버전(2.7.9)이 설치되어 있습니다. 3.x 버전을 설치하기 위해서는 터미널에서 다음과 같은 명령어를 입력해주면 됩니다.

~~~
sudo apt-get update
sudo apt-get install python3
~~~

이렇게 하면 가장 최신 버전의 Python은 아니지만(현재 3.7) 3.4.2 버전의 Python이 설치됩니다. 터미널에서 `python3`라고 입력하면 3.x 버전이 실행됩니다. `pip`의 경우에도 `pip3` 명령어로 실행해야 합니다.