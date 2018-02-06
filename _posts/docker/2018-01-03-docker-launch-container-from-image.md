---
layout: post
title: 컨테이너 실행
category: Docker
permalink: /blog/:year/:month/:day/:title/

tag: [Docker]
---
# Docker Container

Docker 컨테이너는 Docker 이미지로부터 생성할 수 있습니다. 이미지 하나로부터 동일한 컨테이너를 여러개 생성할 수 있으며, 각 컨테이너는 샌드박스(Sandbox) 형태로 되어 있어 각 컨테이너간 간섭이 없습니다.

<br>

## 컨테이너 생성

`docker run <image name>` 명령어를 이용해서 컨테이너를 생성할 수 있습니다. `run` 명령어로 실행한 컨테이너는 주어진 명령이 끝나거나 사용자 명령에 따라 종료됩니다.

예를 들어 다음과 같이 명령을 내리면

~~~
$ docker run ubuntu echo "hello"

hello
~~~

화면에 'hello'라는 메시지만 출력하고 해당 컨테이너는 종료됩니다. 해당 컨테이너의 상태 여부는 `docker ps -a` 명령어를 이용해서 조회 가능합니다.

~~~
docker ps -a
CONTAINER ID        IMAGE                   COMMAND                  CREATED             STATUS                     PORTS               NAMES
303c05fe9c03        ubuntu                  "echo hello"             4 seconds ago       Exited (0) 3 seconds ago                       dreamy_goldstine
~~~

`STATUS`가 `Exited` 상태인 걸 확인할 수 있습니다.

<br>

## 컨테이너 목록 조회

컨테이너 목록은 `docker ps` 명령어를 이용해서 조회할 수 있습니다. 여기에 `-a` 옵션을 붙이면 종료된 컨테이너 정보까지 조회됩니다.

~~~
$ docker ps -a
~~~

<br>

## 컨테이너 시작/재시작/정지

다음 명령어로 컨테이너를 시작하거나 재시작, 정지할 수 있습니다.

~~~
$ docker start <container name>

$ docker restart <container name>

$ docker stop <container name>
~~~

<br>

## 현재 실행중인 컨테이너에 접속하기

`docker attach` 명령어로 현재 실행중인 컨테이너에 접속할 수 있습니다.

~~~
$ docker attach <container name>
~~~

<br>

## 외부에서 컨테이너 안의 명령 실행하기

`docker exec` 명령어로 컨테이너 밖에서 컨테이너 안의 명령을 실행할 수 있습니다.

~~~
$ docker exec <container name> <command>

ex) docker exec snowdeer_docker echo "Hello, SnowDeer"
~~~

<br>

## 컨테이너 삭제

`docker rm` 명령어로 컨테이너 삭제를 할 수 있습니다. 컨테이너 삭제는 `rm` 명령어이며, 이미지 삭제는 `rmi` 입니다.

~~~
$ docker rm snowdeer_docker
~~~