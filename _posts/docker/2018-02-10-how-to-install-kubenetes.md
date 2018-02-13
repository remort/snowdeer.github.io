---
layout: post
title: Kubernetes 설치 방법
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Kubernetes]
---
# Install Kubernetes on Ubuntu

Ubuntu 16.04 LTS 기준으로 Kubernetes를 설치하는 방법입니다. 자세한 내용은 [여기](https://kubernetes.io/docs/setup/independent/install-kubeadm/)를 참고하세요.

<br>

## Docker 설치

Kubernetes를 사용하기 위해서는 먼저 Docker가 설치되어 있어야 합니다. Docker 최신 버전 설치 방법은 [여기](/docker/2018/01/02/install-docker/)에 설명되어 있지만, 안타깝게도 Kubernetes는 Docker 최신 버전을 지원하지 않을 가능성이 높습니다. 따라서 다음 명령어를 이용해서 Kubernetes가 지원하는 버전의 Docker를 설치해야 합니다.

~~~
apt-get update
apt-get install -y docker.io
~~~

또는 Docker CE 17.03 버전을 설치합니다.

~~~
apt-get update
apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository \
   "deb https://download.docker.com/linux/$(. /etc/os-release; echo "$ID") \
   $(lsb_release -cs) \
   stable"
apt-get update && apt-get install -y docker-ce=$(apt-cache madison docker-ce | grep 17.03 | head -1 | awk '{print $3}')

apt-get update && apt-get install -y apt-transport-https
~~~

<br>

## Kubernetes Key 설치

먼저 다음 명령어를 통해 Kubernetes 설치를 위한 Key를 다운로드 합니다.

~~~
sudo curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add 
~~~

<br>

## apt-get 용 Repository 추가

그 다음 `/etc/apt/sources.list.d/kubernetes.list` 파일을 만듭니다. 

~~~
sudo nano /etc/apt/sources.list.d/kubernetes.list
~~~

그리고 위 파일안에 

~~~
deb http://apt.kubernetes.io/ kubernetes-xenial main
~~~

을 추가합니다.

또는 `cat` 명령어를 이용해서 생성할 수도 있습니다.

~~~
cat <<EOF >/etc/apt/sources.list.d/kubernetes.list
deb http://apt.kubernetes.io/ kubernetes-xenial main
EOF
~~~

<br>

## Kubernetes 설치

~~~
apt-get update
apt-get install -y kubelet kubeadm kubectl kubernetes-cni
~~~