---
layout: post
title: Docker 설치하기 (Ubuntu)
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
## Install Docker on Ubuntu

공식 문서는 [여기](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/#set-up-the-repository)를 참조하세요. 

<br>

### 필요 모듈 설치

~~~
$ sudo apt-get update

$ sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
~~~

<br>

### Docker 공식 GPG 키 다운로드

~~~
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
~~~

<br>

### Repository 추가

~~~
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
~~~

<br>

### Docker 설치

~~~
$ sudo apt-get update

$ sudo apt-get install docker-ce
~~~

<br>

### Docker 실행 확인

~~~
$ sudo docker run hello-world
~~~

위 명령어를 실행하면 `hello-world` 이미지를 내려받고 실행할 것입니다.