---
layout: post
title: Docker 실전 예제(Nginx 컨테이너 실행)
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---

# Docker를 이용해서 Nginx 컨테이너 실행하는 예제

## Nginx 이미지 다운로드

<pre class="prettyprint">
$ docker pull nginx:latest
latest: Pulling from library/nginx
8998bd30e6a1: Downloading [=====================>                             ]  13.03MB/30.06MB
6fba654dd4ee: Downloading [======================>
8998bd30e6a1: Pull complete
6fba654dd4ee: Pull complete
661b4150d3a3: Pull complete
13b3c8cc8cde: Pull complete
573040833908: Pull complete
5de8f8b958d2: Pull complete
Digest: sha256:2834dc507516af02784808c5f48b7cbe38b8ed5d0f4837f16e78d00deb7e7767
Status: Downloaded newer image for nginx:latest
docker.io/library/nginx:latest
</pre>

## 다운로드된 이미지 확인

<pre class="prettyprint">
$ docker images
REPOSITORY   TAG       IMAGE ID       CREATED       SIZE
nginx        latest    2e7e2ec411a6   3 weeks ago   134MB
</pre>

## Nginx 컨테이너 실행

PC의 포트는 `5050`, 컨테이너 내부의 포트는 `80` 입니다.

<pre class="prettyprint">
$ docker run --name sample_webserver_1 -d -p 5050:80 nginx:latest
8b0debd502109efc121d28c3de74c1b804580807d9df1e6339afb8b2bfbaf493
</pre>

## 컨테이너 실행 확인

<pre class="prettyprint">
$ docker ps -a
CONTAINER ID   IMAGE          COMMAND                  CREATED          STATUS          PORTS                  NAMES
8b0debd50210   nginx:latest   "/docker-entrypoint.…"   56 seconds ago   Up 56 seconds   0.0.0.0:5050->80/tcp   sample_webserver_1
</pre>

## 호스트의 포트 확인

로컬 웹브라우저에서 `http://localhost:5050` 주소로 접속해서 페이지가 잘 열리는 지 확인합니다.
또는 터미널에서 `curl localhost:5050` 명령어로 확인할 수 있습니다.

## 컨테이너에서 실행 중인 프로세스 출력

<pre class="prettyprint">
$ docker top sample_webserver_1
UID                 PID                 PPID                C                   STIME               TTY                 TIME                CMD
root                2021                1996                0                   11:59               ?                   00:00:00            nginx: master process nginx -g daemon off;
uuidd               2077                2021                0                   11:59               ?                   00:00:00            nginx: worker process
uuidd               2078                2021                0                   11:59               ?                   00:00:00            nginx: worker process
uuidd               2079                2021                0                   11:59               ?                   00:00:00            nginx: worker process
uuidd               2080                2021                0                   11:59               ?                   00:00:00            nginx: worker process
</pre>

## 컨테이너 정지

<pre class="prettyprint">
$ docker stop sample_webserver_1
sample_webserver_1
</pre>