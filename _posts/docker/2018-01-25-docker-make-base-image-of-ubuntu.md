---
layout: post
title: Ubuntu 베이스 이미지 생성하기
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Ubuntu 베이스 이미지 생성하기

Ubuntu 베이스 이미지를 생성하기 위해서는 먼저 우분투 리눅스용 부트스트랩 바이너리 파일들을 받아와야 합니다.

<br>

## debootstrap 설치

부트스트랩 툴인 `debootstrap`를 호스트 PC에서 설치합니다.

~~~
$ sudo apt-get install debootstrap
~~~

<br>

## Ubuntu 바이너리 파일 다운로드

위에서 받은 `debootstrap` 프로그램을 이용해서 Ubuntu 바이너리를 다운로드합니다. 다운로드할 때 Ubuntu의 코드네이(Codename)이 필요한데 [여기](https://wiki.ubuntu.com/Releases)에서 확인할 수 있습니다.

저는 16.04 LTS 버전인 `xenial`로 다운로드했습니다. `debootstrap <code name> <directory name>` 입니다.

~~~
$ sudo debootstrap xenial xenial
~~~

<br>

## docker import 명령어로 베이스 이미지 생성

~~~
$ sudo tar -C xenial -c . | sudo docker import - xenial

sha256:15a945ebf05d877bd5e7e1ec92e8c65726bbed5a46525bf023cda42e29da3502
~~~

이제 베이스 이미지가 생겼고, 컨테이너를 실행해서 확인을 해봅니다.

~~~
$ docker run -i -t --name hello xenial /bin/bash

root@dcfe551d2c01:/# cat /etc/lsb-release 
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=16.04
DISTRIB_CODENAME=xenial
DISTRIB_DESCRIPTION="Ubuntu 16.04 LTS"
~~~