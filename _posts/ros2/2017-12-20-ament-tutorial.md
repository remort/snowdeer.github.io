---
layout: post
title: ROS 2.0 개발 환경 세팅
category: ROS2
tag: [ROS]
---

ROS 2.0을 기반으로 빌드 환경을 설정하는 방법입니다. [여기](https://github.com/ros2/ros2/wiki/Ament-Tutorial)에서 원문을 볼 수 있으며, 저는 제 경험을 토대로 글을 작성했습니다.

# Ament

ROS 1.0에서는 빌드 시스템으로 `catkin`을 이용했었는데, ROS 2.0에서는 `ament`를 이용합니다. `ament`를 사용하기 위해서는 먼저 빌드 환경을 설정해주어야 합니다.

<br>

## 필요한 프로그램 설치

만약 `vcs` 프로그램이 설치되어 있지 않다면 [vcstool](https://github.com/dirk-thomas/vcstool)을 설치합니다.

다음 명령어를 이용해서 'vcstool'을 쉽게 설치할 수 있습니다.

~~~
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

$ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

$ sudo apt-get update

$ sudo apt-get install python3-vcstool

$ source /usr/share/vcstool-completion/vcs.bash
~~~

<br>

## 폴더 생성

아래 명령어를 이용해서 폴더 생성을 해줍니다.

~~~
$ mkdir -p ~/ros2_ws/src
~~~

<br>

## 소스 다운로드

~~~
$ cd ~/ros2_ws

$ wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos

$ vcs import ~/ros2_ws/src < ros2.repos
~~~

<br>

## 빌드

~~~
$ src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
~~~

꽤 오랜 시간이 걸리기 때문에 인내력을 갖고 기다려줍니다. 만약 중간에 응답이 없거나 타임아웃(Timeout)이 발생한 경우는 <kbd>Ctrl</kbd> + <kbd>C</kbd>키를 눌러서 종료한 다음 위 명령어를 다시 입력하면 작업을 이어서 진행합니다.

저 같은 경우는 중간에 너무 오래 걸려서 타임아웃 설정값을 바꿔야 하는 경우도 있었습니다.

<br>

## 설치

빌드가 다 끝나면 `install` 디렉토리에 결과물이 만들어집니다. 다음 명령어를 이용해서 설치를 해줍니다.

~~~
$ . install/local_setup.bash
~~~

<br>

## 샘플 코드 빌드 테스트

이제 `ament`를 이용해서 빌드가 잘 되는지 확인을 해야 합니다. 샘플 코드를 빌드해서 잘 되는지 확인합니다.

~~~
$ mkdir -p ~/ros2_overlay_ws/src

$ cd ~/ros2_overlay_ws/src

$ git clone https://github.com/ros2/examples.git

$ cd ~/ros2_overlay_ws

$ ament build
~~~