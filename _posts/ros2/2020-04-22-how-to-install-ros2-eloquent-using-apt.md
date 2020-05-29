---
layout: post
title: ROS 2.0 Eloquent 설치 방법(apt 이용)
category: ROS2
tag: [ROS]
---

Ubuntu 18.04 기준입니다. 언제부턴가 `apt install` 명령어를 통해 ROS 2.0 설치가 가능해져서 설치가 아주 수월해졌습니다.
`Dashing` 버전과 설치 방법이 동일합니다.

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
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
</pre>

<br>

## ROS2 패키지 설치

<pre class="prettyprint">
sudo apt update
sudo apt install ros-eloquent-desktop
sudo apt install ros-eloquent-ros-base
</pre>

<br>

그리고 ROS2 실행 환경을 실행하려면

<pre class="prettyprint">
source /opt/ros/eloquent/setup.bash
</pre>

와 같은 명령어를 실행하면 되며, `.bashrc` 등에 위 명령어를 추가할 수도 있습니다. 저는 `ZShell`을 사용 중이고 `ROS_DOMAIN_ID` 등을 별도로 지정하기 때문에 `~/.zshrc` 파일에 아래 내용을 설정해놓고 사용합니다.

<pre class="prettyprint">
function ros2env() {
  export ROS_DOMAIN_ID=$1
  source /opt/ros/eloquent/local_setup.zsh
  echo "ROS Domain ID =" $ROS_DOMAIN_ID
}
</pre>

이러면 어디서나 `ros2env 110`과 같은 명령어로 ROS 2.0 환경을 불러올 수 있습니다. 띄어쓰기 뒤의 파라메터는 `ROS_DOMAIN_ID` 값입니다.

<br>

## 부가적 설치

부가적으로 아래 항목들도 설치해놓으면 개발할 때 편리합니다.
특히 가장 아래 부분의 `colcon` 컴파일러는 꼭 설치하는 편이 좋습니다.

<pre class="prettyprint">
sudo apt update
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update

sudo apt install -y libpython3-dev

sudo apt install python3-argcomplete

sudo apt install python3-colcon-common-extensions
</pre>

<br>

## 테스트 명령어

각각 다른 터미널에서 아래 명령어를 실행합니다.

<pre class="prettyprint">
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_py listener
</pre>