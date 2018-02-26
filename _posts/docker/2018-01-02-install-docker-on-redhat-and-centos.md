---
layout: post
title: Docker 설치하기 (RedHat/CentOS)
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
## Install Docker

Linux 버전별 Docker 설치 방법은 다음과 같습니다.

<br>

## Ubuntu

<pre class="prettyprint">
$ sudo apt-get update
$ sudo apt-get install docker.io
</pre>

<br>

## RHEL (RedHat Enterprise Linux) 또는 CentOS 6

RHEL이나 CentOS 6의 기본 패키지 저장소에는 `docker-io` 패키지가 없습니다. 그래서 EPEL(Extra Pacakges for Enterprise Linux)을 먼저 설치해야 합니다.

<pre class="prettyprint">
$ sudo yum install http://dl.fedoraproject.org/pub/epel/6/x86_64/epel-release-6-8.noarch.rpm
$ sudo yum install docker-io
</pre>

만약 AWS의 EC2에 설치하는 RHEL의 경우는 EPEL 저장소를 바로 사용할 수 있기 때문에 `docker-io`만 설치하면 됩니다.

<br>

## CentOS 7

<pre class="prettyprint">
sudo yum install -y yum-utils

sudo yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo

sudo yum makecache fast

sudo yum install -y docker-ce

sudo systemctl start docker

sudo systemctl enable docker

sudo gpasswd -a centos docker
</pre>