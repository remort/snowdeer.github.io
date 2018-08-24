---
layout: post
title: ROS 2.0 Bouncy 설치 방법
category: ROS2
tag: [ROS]
---

ROS 2.0 `Bouncy` 버전 설치 방법입니다. 기존에 `Ardent` 버전이 있었는데, 이번 여름에 Ubuntu 18.04를 지원하면서 `Bouncy` 버전이 새로 나왔습니다. 설치 과정이 조금 달라졌기 때문에 설치 과정을 다시 포스팅합니다.

* `Ardent` 버전: Ubuntu 16.04 지원
* `Bouncy` 버전: Ubuntu 16.04 / Ubuntu 18.04 지원

여기서는 Ubuntu 18.04 기준으로 설명합니다. 하지만, 16.04에서도 설치 과정은 동일합니다. 
공식 페이지는 [여기](https://github.com/ros2/ros2/wiki/Installation)를 보시면 되는데, 아래 포스팅과 조금 차이가 있습니다.

<br>

## Locale 설정

<pre class="prettyprint">
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
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

sudo apt install -y python-rosdep
</pre>

여기에서 맨 아래에 있는 `python-rosdep` 항목 설치가 실패한다면 `python-rosdep2`로 이름 변경해서 설치를 시도해보세요. 

<br>

## Test를 위한 pip Package 설치

<pre class="prettyprint">
sudo -H python3 -m pip install -U \
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
  pytest-repeat \
  pytest-rerunfailures

python3 -m pip install -U \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
</pre>

<br>

## Fast-RTPS 라이브러리 설치

<pre class="prettyprint">
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
</pre>

## ROS2 설치 파일 다운로드

[여기](https://github.com/ros2/ros2/releases)에서 필요한 바이너리 설치 파일을 다운로드합니다. 자신의 Ubuntu 버전에 따라서 선택하시면 됩니다. 참고로 `bionic`가 18.04, `xenial`이 16.04 입니다.

<br>

## 다운로드한 ROS2 파일 설치

<pre class="prettyprint">
mkdir -p ~/ros2_install
cd ~/ros2_install
tar xf ~/Downloads/ros2-bouncy-linux-bionic-x86_64.tar.bz2
</pre>

<br>

## ROS2 Dependency 설치

<pre class="prettyprint">
sudo apt install -y python-rosdep
sudo rosdep init
sudo rosdep update

sudo rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 osrf_testing_tools_cpp poco_vendor rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"
</pre>

<br>

## RTI Connext 설정

<pre class="prettyprint">
export RTI_LICENSE_FILE=path/to/rti_license.dat

sudo apt update && sudo apt install -q -y \
        rti-connext-dds-5.3.1
</pre>

여기서 주의할 점은 RTI Connext 설치 도중 긴 스크롤 메시지를 읽고 `yes`를 타이핑해야 하는 절차가 있다는 점입니다. 

<br>

## 예제 파일 실행

창 2개를 띄워놓고 다음 명령어 실행해서 잘 동작하는지 확인해봅니다.

<pre class="prettyprint">
. ~/ros2_install/ros2-linux/setup.bash
ros2 run demo_nodes_cpp talker

. ~/ros2_install/ros2-linux/setup.bash
ros2 run demo_nodes_cpp listener
</pre>

