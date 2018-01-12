---
layout: post
title: Docker 기본 명령어들
category: Docker
permalink: /blog/:year/:month/:day/:title/

tag: [Docker]
---
# Docker 기본 명령어들

<br>

## 이미지 검색

`docker search` 명령어로 Docker Hub에서 이미지를 검색할 수 있습니다.

~~~
$ sudo docker search ubuntu
~~~

<br>

## 이미지 다운로드

`docker pull` 명령어로 이미지를 받아올 수 있습니다.

~~~
$ sudo docker pull ubuntu:latest
~~~

위 코드로 수행할 경우 Ubuntu 최신 버전을 가져오게 되는데, `docker pull ubuntu:14.04`와 같은 형태로 태그를 지정하여 원하는 버전을 받을 수도 있습니다.

<br>

## 이미지 목록 확인

`docker images` 명령어로 현재 받은 이미지 리스트를 출력할 수 있습니다.

~~~
$ sudo docker images
~~~

<br>

## 컨테이너 생성

`docker run <옵션> <이미지 이름> <실행 파일 이름>` 명령어를 이용해서 컨테이너를 실행할 수 있습니다. 아래 예제는 ubuntu 이미지 상의 `/bin/bash` 실행 파일을 실행하도록 하는 명령입니다. `--name` 옵션으로 컨테이너의 이름을 지정할 수 있습니다.

~~~
$ sudo docker run -i -t --name snow_docker ubuntu /bin/bash
~~~

<br>

## 컨테이너 목록 확인

`docker ps` 명령어를 이용하면 컨테이너 리스트를 볼 수 있습니다. `-a` 옵션으로 현재 정지 상태의 컨테이너 목록들도 볼 수 있습니다.

~~~
$ sudo docker ps -a
~~~

<br>

## 컨테이너 시작/재시작/정지

다음 명령어로 컨테이너를 시작하거나 재시작, 정지할 수 있습니다.

~~~
$ sudo docker start snow_docker

$ sudo docker restart snow_docker

$ sudo docker stop snow_docker
~~~

<br>

## 현재 실행중인 컨테이너에 접속하기

`docker attach` 명령어로 현재 실행중인 컨테이너에 접속할 수 있습니다.

~~~
$ sudo docker attach snow_docker
~~~

<br>

## 외부에서 컨테이너 안의 명령 실행하기

`docker exec` 명령어로 컨테이너 밖에서 컨테이너 안의 명령을 실행할 수 있습니다.

~~~
$ sudo docker exec snow_docker echo "Hello, SnowDeer"
~~~

<br>

## 컨테이너 삭제

`docker rm` 명령어로 컨테이너 삭제를 할 수 있습니다.

~~~
$ sudo docker rm snow_docker
~~~

<br>

## 이미지 삭제

이미지 삭제는 `docker rmi` 명령어로 할 수 있습니다.

~~~
$ sudo docker rmi ubuntu:latest
~~~
