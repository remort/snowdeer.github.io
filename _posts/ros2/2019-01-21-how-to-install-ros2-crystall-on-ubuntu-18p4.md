---
layout: post
title: ROS 2.0 Crystal 설치 방법
category: ROS2
tag: [ROS]
---

ROS 2.0 `Crystal` 버전이 새로 나왔습니다. 역시나 기존의 `Ardent`나 `Bouncy` 버전 때와 설치 방법이 조금 다른데, 그래도 거의 `Bouncy` 때의 설치 과정과 유사합니다. 

아래 포스팅은 Ubuntu 18.04 버전을 처음 설치했다고 가정하고(아무런 패키지가 설치되지 않은 상태), ROS 2.0 Crystal 버전을 설치하는 과정입니다.

<br>

## Locale 설정

<pre class="prettyprint">
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
</pre>

<br>

## 기본 패키지 설치 및 repo key 설치

<pre class="prettyprint">
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
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

## 기본 컴포넌트들 설치

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

## ROS 2.0 바이너리 다운로드 및 설치

미리 빌드된 ROS 2.0 바이너리를 [다운로드](https://github.com/ros2/ros2/releases)합니다.
그리고 아래 명령어로 압축해제를 합니다.

<pre class="prettyprint">
mkdir -p ~/ros2_install
cd ~/ros2_install
tar xf ~/Downloads/ros2-crystal-20190117-linux-bionic-amd64.tar.bz2
</pre>

<br>

## rosdep 설치 및 초기화

<pre class="prettyprint">
sudo apt install -y python-rosdep

sudo rosdep init
sudo rosdep fix-permissions
rosdep update
</pre>

만약 중간에 `catkin-pkg` 버전 때문에 에러가 발생하면 아래 명령어를 이용해서 버전 업데이트를 해줍니다.

<pre class="prettyprint">
sudo apt-get update && sudo apt-get install --only-upgrade python-catkin-pkg
</pre>

<br>

## OpenSlice 및 RTI Connext

`OpenSlice`와 `RTI Connext`는 굳이 설치를 할 필요는 없습니다. (어차피 상용 License라 사용하기엔 애매합니다.)

여기서는 패스합니다.

<br>

## 설치 확인

창을 2개 열어서 다음 명령어가 잘 동작하는지 확인합니다.~~~~

<pre class="prettyprint">
. ~/ros2_install/ros2-linux/setup.bash
ros2 run demo_nodes_cpp talker
</pre>

<br>

<pre class="prettyprint">
. ~/ros2_install/ros2-linux/setup.bash
ros2 run demo_nodes_cpp listener
</pre>