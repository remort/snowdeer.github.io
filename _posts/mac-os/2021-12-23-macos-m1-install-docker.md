---
layout: post
title: M1에 Docker 설치하는 방법

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# M1에 Docker 설치하는 방법

## Docker 다운로드
Docker가 M1에서 동작할 수 있도록 정식 버전이 릴리즈되어 있습니다.

![image](/assets/tips-mac/019.png)

[여기](https://docs.docker.com/desktop/mac/install/)에서 다운로드할 수 있습니다.

## Docker Subscription Service Agreement

![image](/assets/tips-mac/020.png)

Accept를 누르고 계속 진행합니다.

## Tutorial

설치를 하고나면 간단한 튜토리얼을 해볼 수 있습니다.

![image](/assets/tips-mac/021.png)

`clone`, `build`, `run`, `share`에 대한 간단한 명령어를 확인할 수 있습니다.

## 터미널에서 확인
터미널에서도 Docker 명령어가 잘 동작하는 것을 확인합니다.

![image](/assets/tips-mac/022.png)

## 실제 동작 확인 
여기서는 RabbitMQ를 이용해서 테스트 해보도록 하겠습니다.
아래 명령어를 입력하면 RabbitMQ 이미지까지 자동으로 다운로드 후 실행까지 합니다.

<pre class="prettyprint">
$ docker run -d --hostname rabbit --name rabbit -p 15672:15672 -p 5672:5672 rabbitmq:3-management

Unable to find image 'rabbitmq:3-management' locally
3-management: Pulling from library/rabbitmq
a39c84e173f0: Pull complete
7d3994c28245: Pull complete
10c911d5c079: Pull complete
769f1e4dc40b: Pull complete
2090f091d001: Pull complete
f9e692861b3e: Pull complete
d26c4b0e32ac: Pull complete
3cf30fecd6f0: Pull complete
c5b6ca5b444e: Pull complete
aa1dff4734e4: Pull complete
Digest: sha256:4c4b66ad5ec40b2c27943b9804d307bf31c17c8537cd0cd107236200a9cd2814
Status: Downloaded newer image for rabbitmq:3-management
501592f6fdc53e1da8ff527b2b4cf7853ce28bf05dd06e3c62fbac6e747e4945
</pre>

잘 동작하는지 확인해봅니다. 

<pre class="prettyprint">
$ docker ps -a

CONTAINER ID   IMAGE                   COMMAND                  CREATED          STATUS          PORTS                                                                                                         NAMES
501592f6fdc5   rabbitmq:3-management   "docker-entrypoint.s…"   28 seconds ago   Up 27 seconds   4369/tcp, 5671/tcp, 0.0.0.0:5672->5672/tcp, 15671/tcp, 15691-15692/tcp, 25672/tcp, 0.0.0.0:15672->15672/tcp   rabbit
</pre>

그리고 사파리나 크롬 등에서 [http://localhost:15672/](http://localhost:15672/)에 접속해서 다음 화면이 잘 나오는지 확인합니다.

![image](/assets/tips-mac/023.png)