---
layout: post
title: ROS 2.0 설치 방법
category: ROS
tag: [ROS]
---

ROS 2.0을 설치하는 방법입니다. [ROS 2.0 공식 페이지](https://github.com/ros2/ros2/wiki/Installation)에서 더 자세히 볼 수 있으며, 여기서는 제 경험을 토대로 다시 정리했습니다.

# ROS 2.0 설치

저는 Ubuntu 16.04 LTS에 ROS 2.0을 설치했습니다.

## 필요한 라이브러리 설치

다음 명령어를 이용해서 필요한 라이브러리들을 먼저 설치합니다.

~~~
$ sudo apt-get update && sudo apt-get install -q -y \
     libopencv-core2.4v5 \
     libhighgui2.4 \
     libopencv-imgproc2.4v5 \
     libasio-dev \
     libeigen3-dev \
     libtinyxml-dev \
     libtinyxml2-dev \
     libcurl4-openssl-dev \
     libqt5core5a \
     libqt5gui5 \
     libqt5opengl5 \
     libqt5widgets5 \
     libxaw7-dev \
     libgles2-mesa-dev \
     libglu1-mesa-dev \
     python3-pip \
     python3-setuptools \
     python3-yaml \
     wget \
     tree

$ sudo apt-get update && sudo apt-get install -q -y \
    libboost-thread-dev

$ sudo pip3 install argcomplete
~~~

<br>

## ROS 2.0 바이너리 패키지 다운로드

다음 경로에서 ROS 2.0 최신 바이너리 패키지를 다운 받습니다.

* [https://github.com/ros2/ros2/releases](https://github.com/ros2/ros2/releases)

저는 'ros2-ardent-package-linux-fastrtps-opensplice-x86_64.tar.bz2' 파일을 다운받았습니다.

그런 다음 다음 명령어를 통해 압축 파일을 해제합니다.

~~~
$ mkdir -p ~/ros2_install

$ cd ~/ros2_install

$ tar xf ~/Downloads/ros2-package-linux-x86_64.tar.bz2
~~~

<br>

## 샘플 테스트

이제 잘 설치가 되었고 동작이 되는지 확인하기 위해서 터미널에서 다음 명령어를 실행해봅니다.

~~~
$ . ~/ros2_install/ros2-linux/setup.bash

$ ros2 run demo_nodes_cpp talker
~~~

~~~
$ . ~/ros2_install/ros2-linux/setup.bash

$ ros2 run demo_nodes_cpp listener
~~~

<br>

잘 동작이 되는 걸 확인했으면, `.bashrc` 파일 맨 끝에 아래 항목을 추가합니다.

~~~
. ~/ros2_install/ros2-linux/setup.bash
~~~