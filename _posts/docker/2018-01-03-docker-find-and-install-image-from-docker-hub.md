---
layout: post
title: Docker Hub로부터 이미지 검색 및 다운로드
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker Hub

[Docker Hub](https://hub.docker.com/)에는 10만개가 넘는 Docker 이미지들이 존재합니다. 직접 Docker Hub 홈페이지를 방문해도 되고 터미널에서 커맨드 명령어로 원하는 이미지를 검색하거나 다운로드할 수 있습니다.

여기서는 터미널에서 커맨드 명렁어로 이미지를 검색하고 다운로드하는 방법을 포스팅합니다.

<br>

## 이미지 검색

Docker 이미지는 다음 명령어를 이용해서 검색할 수 있습니다.

~~~
$ docker search <image name>

ex) docker search ubuntu
~~~

<br>

## 이미지 다운로드

`pull` 명령어를 이용해서 원하는 이미지를 다운로드할 수 있습니다. 이미지 이름 뒤의 `<tag>`를 이용하여 원하는 버전의 이미지를 다운로드할 수 있으며, 생략할 경우 기본값으로 `latest` 태그가 입력됩니다.

~~~
$ docker pull <image name>:<tag>

ex) docker pull ubuntu
~~~

또는 `run` 명령어를 이용해서 원하는 이미지를 다운로드하면서 해당 이미지의 컨테이너를 실행할 수 있습니다. `run` 명령어는 로컬에 이미지가 있을 경우 해당 이미지로부터 컨테이너를 생성하며, 로컬에 이미지가 없을 경우 이미지를 다운로드합니다.

~~~
$ docker run -i -t --name <container name> <image name>

ex) docker run -i -t --name hello ubuntu
~~~

<br>

## 이미지 리스트 조회

`docker images` 명령어를 이용해서 로컬에 설치된 이미지 리스트를 확인할 수 있습니다.

~~~
$ docker images

REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
ubuntu              latest              0458a4468cbc        10 days ago         112MB
hello-world         latest              f2a91732366c        2 months ago        1.85kB
~~~

<br>

## 이미지 삭제

이미지 삭제는 `rmi` 명령어를 이용합니다.

~~~
$ docker rmi <image name>:<tag>

ex) docker rmi ubuntu:latest
~~~