---
layout: post
title: SSH 설치 방법
category: Linux
tag: [Linux]
---
# SSH Server 설치 방법

<br>

## 설치

다음 명령어를 이용해서 설치할 수 있습니다.

~~~
sudo apt-get install openssh-server
~~~

<br>

## 재실행

보통 설치와 함께 서비스가 같이 실행되기 때문에 다른 설정은 필요없습니다. 하지만, 만약 ssh 서버를 재시작하고 싶으면 다음 명령어를 사용하면 됩니다.

~~~
sudo /etc/init.d/ssh restart
~~~

<br>

## 포트 변경

~~~
sudo nano /etc/ssh/sshd_config 
~~~