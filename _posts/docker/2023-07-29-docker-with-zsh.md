---
layout: post
title: Ubuntu(zsh) 이미지 실행하는 방법
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---

# Docker 컨테이너에 zsh 설치하기

## 이미 빌드된 이미지 사용하기

이미 Docker Hub에 위 이미지가 올라가 있기 때문에, 별도의 과정 필요없이 아래 명령어로 바로 사용할 수 있습니다.

<pre class="prettyprint">
$ docker run -it snowdeer/ubuntu-22p04 zsh
</pre>

## 이미지 생성하는 방법

위의 이미지를 생성하는 방법입니다. Dockerfile을 이용해서 생성할 수도 있지만,
실제 Ubuntu를 사용하면서 신규 패키지를 설치하는 경우도 있기 때문에 이번에는 컨테이너에 다양한 패키지를
직접 설치한다음 이미지로 만드는 방법으로 실행해보았습니다.

zsh가 설치된 Docker 이미지는 이미 [Docker Hub](https://hub.docker.com/r/frapsoft/zsh)에서 받을 수 있지만,
제가 개인적으로 사용하는 Ubuntu 이미지에 zsh를 설치해서 사용하는 것을 더 좋아합니다.

기본 ubuntu 이미지에 `zsh`만 설치했을 때는 몇 가지 문제(ex. Locale 설정 등)가 발생해서 그 해결 방법을
아래와 같이 포스팅합니다.

## Ubuntu 이미지 다운로드

<pre class="prettyprint">
# 최신 Ubuntu 이미지 다운로드(현 시점 기준 22.04 버전)
$ docker pull ubuntu

$ docker run -it ubuntu
</pre>

## 필요 패키지 설치

Docker 컨테이너 속의 터미널에서는 다음과 같이 입력합니다.

<pre class="prettyprint">
$ apt update

# vi를 써도 되지만 개인적으론 nano가 더 편해서 nano도 설치
$ apt install -y git curl nano zsh

# 생략해도 되지만 network 관련 유틸리티이기 때문에 그냥 설치 
$ apt install -y iputils-ping net-tools iproute2 dnsutils

# oh-my-zsh 설치
$ sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"

... 
Time to change your default shell to zsh:
Do you want to change your default shell to zsh? [Y/n] y
</pre>

## ZSH 설정 편집

`nano ~/.zshrc` 명령어로 편집기를 실행해서 `ZSH_THEME="robbyrussell"` 부분만 `ZSH_THEME="agnoster"`으로
수정합니다.

이 상태에서 `zsh` 명령어로 zshell을 실행하면 아래와 같은 오류가 발생합니다.

<pre class="prettyprint">
$ zsh

(anon):12: character not in range
</pre>

## ZSH Locale 문제 해결

이 문제는 Locale 때문에 발생하는 문제로 아래와 같이 해결할 수 있습니다.

<pre class="prettyprint">
$ apt install -y locales

$ locale-gen en_US.UTF-8
</pre>

자, 제가 개인적으로 즐겨 사용하는 ubuntu 이미지가 생성 완료되었습니다.
이 상태에서 해당 컨테이너의 내용으로 이미지를 만들어줍니다.
(컨테이너 터미널 외부에서 입력합니다.)

## Docker 이미지 commit

<pre class="prettyprint">
# 먼저 컨네이터의 ID를 확인합니다.
$ docker ps -a
CONTAINER ID   IMAGE     COMMAND       CREATED         STATUS                     PORTS     NAMES
d0b9b5314e29   ubuntu    "/bin/bash"   8 minutes ago   Exited (0) 2 seconds ago             adoring_carson

$ docker commit d0b9 snowdeer/ubuntu-22p04
sha256:b4a5a887e027af8a49062c67e2a53ea6cac8fd0c22d92a013ec2aba3b40896dc

$ docker images
REPOSITORY                    TAG       IMAGE ID       CREATED         SIZE
snowdeer/ubuntu-22p04         latest    b4a5a887e027   3 seconds ago   226MB
</pre>

## Docker Hub에 업로드

<pre class="prettyprint">
$ docker push snowdeer/ubuntu-22p04
Using default tag: latest
The push refers to repository [docker.io/snowdeer/ubuntu-22p04]
f5048f19797c: Pushed
c5ca84f245d3: Pushed
latest: digest: sha256:206d44f8646bb6bd041728531b74e193f325c340c8537b4881a3daf2f0522349 size: 741
</pre>
