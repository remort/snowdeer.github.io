---
layout: post
title: Docker 설치하기
category: Docker
permalink: /blog/:year/:month/:day/:title/

tag: [Docker]
---
# Docker 설치

Docker는 기본적으로 Linux를 사용합니다. 즉, Linux 없이는 동작할 수 없습니다. Windows나 Mac에서 구동시키기 위해서는 리눅스 가상 머신부터 설치한 다음 Docker를 실행해야 합니다.

<br>

## Windows 10에서의 Docker

다행히 Windows 10에서는 Ubuntu를 스토어에서 다운받을 수 있습니다. 즉, 별도의 [VirtualBox](https://www.virtualbox.org/) 등의 프로그램 없이 Docker를 설치할 수 있습니다. MacOS 같은 경우는 VirtualBox를 이용해서 Ubuntu 등의 Linux를 먼저 설치하고 Docker를 설치하면 됩니다. (다만 설치는 되지만, 실행은 안될 것입니다. WSL(Windows Subsystem for Linux)의 호환성 때문인데, [여기](https://blogs.technet.microsoft.com/virtualization/2017/12/08/wsl-interoperability-with-docker/)에서 해결법을 볼 수 있습니다. 저는 잘 안되서, 그냥 Linux에서 Docker를 사용하고 있습니다. ㅜ_ㅜ)

콘솔에서 다음 명령어를 입력합니다.

<br>

## Docker 설치 명령어

Ubuntu 기준으로 다음 명령어를 터미널 상에서 입력합니다.

~~~
$ sudo apt-get update

$ sudo apt-get install docker.io
~~~

설치 후 `docker` 명령어가 잘 실행되는지 확인합니다.
