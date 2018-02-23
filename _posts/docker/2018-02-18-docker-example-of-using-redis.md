---
layout: post
title: Docker 실습 예제 (Redis)
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Redis 공식 이미지 사용

## Redis 이미지 다운로드(pull)

<pre class="prettyprint">
$ docker pull redis

Using default tag: latest
latest: Pulling from library/redis
d2ca7eff5948: Pull complete
1d1a2245aaa6: Pull complete
9a483dd2a28b: Pull complete
7b78ebdc44f0: Pull complete
79b76500ef26: Pull complete
b2991d9a5624: Pull complete
Digest: sha256:e55dff3a21a0e7ba25e91925ed0d926d959dac09f9099fd1bcc919263305f1e4
Status: Downloaded newer image for redis:latest
</pre>

<br>

## Redis 컨테이너 시작

앞에서 가져온 `redis` 임지를 이용해서 컨테이너를 시작합니다.

<pre class="prettyprint">
$ docker run --name myredis -d redis

a16226308354840de9dadda78bb83f2378dc5cdb0270c097942357b466bf023f
</pre>

`-d` 옵션을 주면 컨테이너를 백그라운드에서 실행시킬 수 있습니다.

<br>

## 컨테이너간 연결

`redis` 컨테이너를 시작하긴 헀지만, 그 이후에 해야 할 일들이 있습니다. 어떻게든 데이터베이스로 연결을 해야 합니다.

`redis-cli` 도구를 설치하기 위해서 새로운 컨테이너를 생성하고 앞서 생성한 컨테이너와 연결을 해줍니다.

<pre class="prettyprint">
$ docker run --rm -it --link myredis:redis redis /bin/bash

root@c5706ca49d45:/data# redis-cli -h redis -p 6379
redis:6379> ping
PONG

redis:6379> set "abc" 123
OK

redis:6379> get "abc"
"123"

redis:6379> exit

root@c5706ca49d45:/data# exit

exit
</pre>

`--link` 옵션을 이용하게 되면 새로운 도커 컨테이너와 기존의 `myredis` 이름의 컨테이너가 연결이 됩니다. 새로운 컨테이너 안에서 `myredis` 컨테이너를 `redis`라는 이름으로 참조합니다. 이러한 작업을 수행하려면 도커는 컨테이너 `/etc/hosts`에서 `redis`를 위한 진입점을 생성하고, `myredis` 컨테이너의 IP 주소를 가리키도록 합니다. 

이렇게 하면 Redis 컨테이너의 IP 주소를 전달하거나 찾을 필요없이 `redis-cli`에서 `redis`라는 호스트 이름을 사용할 수 있게 됩니다.

<br>

## 볼륨 설정

컨테이너가 종료되면 컨테이너 내부의 데이터는 사라집니다. 따라서 컨테이너안의 데이터를 유지하고 백업하기 위해서는 호스트 또는 다른 컨테이너 간에 데이터를 공유할 수 있는 설정이 필요합니다. 도커에서는 볼륨(Volume)이라는 개념을 통해 컨테이너에 파일 또는 디렉토리를 마운트(Mount)할 수 있습니다.

도커에서 볼륨을 사용하는 방법은 다음과 같이 두 가지 방법이 있습니다.

* Dockerfile 안에 `VOLUME` 설정을 사용
* `docker run` 명령어를 사용할 때 `-v` 옵션을 활용

예를 들어 컨테이너 안에 `/data`라는 볼륨을 생성하고 싶으면 다음과 같이 할 수 있습니다.

<pre class="prettyprint">
VOLUME /data
</pre>

또는

<pre class="prettyprint">
docker run -v /data redis
</pre>

기본적으로 디렉토리나 파일은 호스트의 도커 설치 디렉토리(보통은 `/var/lib/docker/`)안에 마운트됩니다. `-v` 옵션을 이용해서 호스트의 특정 디렉토리를 컨테이너 안의 디렉토리로 마운트 할 수도 있습니다.

<pre class="prettyprint">
docker run -v [호스트 디렉토리]:[컨테이너 디렉토리] reids

ex) docker run -v /home/snowdeer/docker/data:/data redis
</pre>

<br>

## 데이터 백업

앞서 만들었던 `myredis` 컨테이너가 실행 중일 때 백업을 하고 싶으면 다음 명령을 이용해서 백업을 할 수 있습니다.

<pre class="prettyprint">
$ docker run --rm --volumes-from myredis -v $(pwd)/redis-backup:/backup debian cp /data/dump.rdb /backup/
</pre>

위 명령을 수행하면 현재 호스트 디렉토리에 `redis-backup`이라는 디렉토리를 만들고 컨테이너 안의 `dump.rdb` 파일을 호스트로 백업하게 됩니다.

<br>

## 컨테이너 종료와 삭제

컨테이너 종료와 삭제 방법은 다음과 같습니다.

<pre class="prettyprint">
$ docker stop myredis

$ docker rm myredis
</pre>

만약 모든 컨테이너들을 삭제하고 싶으면 다음과 같이 명령어를 내리면 됩니다.

<pre class="prettyprint">
$ docker stop $(docker ps -aq)

$ docker rm $(docker ps -aq)
</pre>