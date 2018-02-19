---
layout: post
title: Docker Hub에 이미지 업로드
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker Hub에 이미지 업로드하는 방법

먼저 [Docker Hub](https://hub.docker.com/)에 회원가입이 되어 있어야 합니다.

<br>

## Docker 로그인

로그인은 `docker login` 명령어를 이용해서 할 수 있습니다.

~~~
$ docker login

Login with your Docker ID to push and pull images from Docker Hub. If you don't have a Docker ID, head over to https://hub.docker.com to create one.
Username: snowdeer
Password:
Login Succeeded
~~~

<br>

## 샘플 이미지 생성

Docker Hub에 이미지를 올릴 때는 이미지 이름을 `[Docker Hub 사용자 계정]/[이미지 이름]:[태그]` 형태로 생성해야 합니다.

Docker 이미지를 하나 생성해봅니다.

<br>

### server.js

<pre class="prettyprint">
var http = require('http');

var handleRequest = function(request, response) {
  console.log('Received request for URL: ' + request.url);
  response.writeHead(200);
  response.end('Hello SnowDeer!');
};
var www = http.createServer(handleRequest);
www.listen(8080);
</pre>

<br>

### Dockerfile 생성

그리고 다음과 같은 Dockerfile 파일을 생성합니다.

<pre class="prettyprint">
FROM node:6.9.2
EXPOSE 8080
COPY server.js .
CMD node server.js
</pre>

<br>

### Docker Image 빌드

다음 명령어를 이용해서 Docker 이미지를 빌드합니다.

~~~
docker build -t snowdeer/hello-nodejs:v1 .
~~~

<br>

## Docker Hub에 push

~~~
$ docker push snowdeer/hello-nodejs:v1

The push refers to a repository [docker.io/snowdeer/hello-nodejs]
9b0e32122635: Pushed
381c97ba7dc3: Mounted from library/node
604c78617f34: Mounted from library/node
fa18e5ffd316: Mounted from library/node
0a5e2b2ddeaa: Mounted from library/node
53c779688d06: Mounted from library/node
60a0858edcd5: Mounted from library/node
b6ca02dfe5e6: Mounted from library/node
v1: digest: sha256:64a2d069b8a1d5173ce7b6e6eadfec183ab63ae7857ce7ec73bfd9758fb812c0 size: 2002
~~~