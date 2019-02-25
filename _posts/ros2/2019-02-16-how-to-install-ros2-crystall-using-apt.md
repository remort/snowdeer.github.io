---
layout: post
title: ROS 2.0 Crystal 설치 방법(apt 이용)
category: ROS2
tag: [ROS]
---

ROS 2.0 Crystal 버전부터는 Ubuntu의 `apt install` 명령어를 이용해서 더욱 더 간편하게 설치할 수 있게 되었습니다.

다음 포스팅은 Ubuntu 18.04 기준이며, 포맷 후 처음 설치하는 과정입니다.

<br>

## Locale 설정

<pre class="prettyprint">
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
</pre>

<br>

## GPG Key 설치

<pre class="prettyprint">
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
</pre>

<pre class="prettyprint">
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
</pre>

<br>

## vcs-tool 설치

<pre class="prettyprint">
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 0xB01FA116
sudo apt-get update
sudo apt-get install python3-vcstool
</pre>

<br>

## 기본 컴포넌트 설치

<pre class="prettyprint">
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  wget

sudo apt install -y libpython3-dev

python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \

  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  git+https://github.com/lark-parser/lark.git@0.7d \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
</pre>

<br>

## ROS 2 패키지 설치

<pre class="prettyprint">
export CHOOSE_ROS_DISTRO=crystal  # or bouncy or ardent
sudo apt update

sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop
sudo apt install ros-$CHOOSE_ROS_DISTRO-ros-base
</pre>

그리고 ROS2 실행 환경을 만들려면 

<pre class="prettyprint">
source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash
</pre>

와 같은 명령어를 실행하면 되며, `.bashrc` 등에 위 명령어를 추가할 수도 있습니다.

<br>

## OpenSplice 및 RTI Connext 설치

옵션입니다. ROS2는 기본적으로 `fast-rtps`를 사용하기 때문에 OpenSplice나 RTI Connext는 추가적으로 설치할 필요는 없습니다. 하지만 경고 메시지 등이 거슬려서 설치를 할 경우에는 아래와 같은 명령어를 이용해서 설치할 수 있습니다.

<pre class="prettyprint">
sudo apt update
sudo apt install ros-$ROS_DISTRO-rmw-opensplice-cpp # for OpenSplice
sudo apt install ros-$ROS_DISTRO-rmw-connext-cpp # for RTI Connext (requires license agreement)
</pre>