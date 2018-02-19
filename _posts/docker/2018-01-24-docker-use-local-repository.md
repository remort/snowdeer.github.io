---
layout: post
title: Docker Local Repository 사용하는 방법
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker Local Repository 사용하는 방법

현재 사용하는 PC에 Docker Local Repository를 구성하고 싶으면 아래의 방법대로 하시면 됩니다.

## Docker init.d 파일 수정

먼저 Docker 서비스를 종료합니다.

~~~
$ sudo service docker stop
~~~

그 이후 `/etc/init.d/docker` 파일을 편집기로 열어서 `DOCKER_OPTS` 항목에 아래 내용을 추가해줍니다.

~~~
DOCKER_OPTS=--insecure-registry localhost:5000
~~~

그런 다음 Docker 서비스를 다시 실행합니다.

~~~
$ sudo service docker restart
~~~

<br>

## Registry Server 이미지 다운로드

Docker Registry 서버도 Docker Hub에서 이미지로 배포하고 있습니다. 다음 명령어로 Registry 이미지를 다운로드합니다.

~~~
$ docker pull registry:latest
~~~

<br>

## Registry 컨테이너 실행

로컬 PC의 `/home/snowdeer/Docker/registry' 폴더를 컨테이너에 연결시켰습니다.

~~~
docker run -d -p 5000:5000 --name snowdeer-registry -v /home/snowdeer/Docker/registry:/tmp/registry registry
~~~

<br>

## 로컬 Registry에 Docker 이미지 올리기

먼저 `commit` 명령어를 이용해서 Docker 이미지를 간단하게 만들어봅니다.

~~~
$ docker run --name snow-nginx nginx

$ docker commit -m "Snow Nginx" snow-nginx snow-nginx:0.1
~~~

그런 다음 위에서 만들었던 로컬 Registry에 해당 이미지를 `push` 명령어를 이용해서 업로드합니다.

~~~
$ docker tag snow-nginx:0.1 localhost:5000/snow-nginx:0.1

$ docker push localhost:5000/snow-nginx:0.1

The push refers to repository [localhost:5000/snow-nginx]
fced5b294318: Pushed 
a103d141fc98: Pushed 
73e2bd445514: Pushed 
2ec5c0a4cb57: Pushed 
0.1: digest: sha256:6e7f42cb6b7df0c8a4cbd75d0fbecefcafdcc6c0d95bb4e07a19c05ef19e5171 size: 1155
~~~

태그를 생성하는 명령은 `docker tag <image name>:<tag> <Docker registry URL>/<image name>:<tag>` 입니다.

이미지 업로드는 `docker push <Docker registry URL>/<image name>:<tag>` 입니다.

개인이 만든 Registry에 이미지를 올릴 때는 먼저 태그를 생성해야 하며, 그 이후 `push` 명령을 사용할 수 있습니다.

<br>

## 로컬 Registry의 이미지 리스트 조회

로컬 Registry의 이미지도 `docker images` 명령을 이용해서 조회할 수 있습니다.

~~~
$ docker images

REPOSITORY                  TAG                 IMAGE ID            CREATED             SIZE
localhost:5000/snow-nginx   0.1                 e3fc966b1d25        5 minutes ago       108MB
snow-nginx                  0.1                 e3fc966b1d25        5 minutes ago       108MB
ubuntu                      latest              0458a4468cbc        11 days ago         112MB
wordpress                   latest              e8cebf03929c        2 weeks ago         407MB
mysql                       latest              f008d8ff927d        3 weeks ago         409MB
registry                    latest              d1fd7d86a825        3 weeks ago         33.3MB
nginx                       latest              3f8a4339aadd        5 weeks ago         108MB
~~~

<br>

## 로컬 Registry의 이미지 다운로드

~~~
$ docker pull localhost:5000/snow-nginx:0.1
~~~

<br>

## 로컬 Registry의 이미지 삭제

~~~
$ docker rmi localhost:5000/snow-nginx:0.1
~~~