---
layout: post
title: MAC OS에 ROS2 설치하는 방법
category: ROS2
tag: [ROS]
---
# MAC OS에 ROS2 설치하는 방법

OS 2.0 설치 방법은 자주 업데이트되거나 변경되기 때문에 아래 방법이 잘 되지 않으면 [공식 홈페이지](https://index.ros.org/doc/ros2/Installation/)를 참고하는 편이 좋습니다.
2018년 11월 5일 기준 제가 직접 설치하면서 검증한 방법입니다.

먼저 Homebrew나 XCode CLT(Command Line Tool)는 설치가 되어 있어야 합니다.
다음 명령어를 이용해서 각각을 설치할 수 있습니다.

<pre class="prettyprint">
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

xcode-select --install
</pre>

<br>

그 이후 아래와 같이 필요한 컴포넌트들을 설치해줍니다.

<pre class="prettyprint">
brew install python3

# install asio and tinyxml2 for Fast-RTPS
brew install asio tinyxml2

# install dependencies for robot state publisher
brew install tinyxml eigen pcre poco

# OpenCV isn't a dependency of ROS 2, but it is used by some demos.
brew install opencv

# install OpenSSL for DDS-Security
brew install openssl

# install Qt for RViz
brew install qt freetype assimp
</pre>

<br>

`pip` 명령어를 이용해서 다음 컴포넌트들도 설칲해줍니다.

<pre class="prettyprint">
python3 -m pip install catkin_pkg empy pyparsing pyyaml setuptools argcomplete
</pre>

<br>

## ROS 2 binary 다운로드

여기서는 그냥 ROS 2.0 바이너리 파일을 다운받아서 압축을 풀고 설치해줍니다.
바이너리 파일은 [여기](https://github.com/ros2/ros2/releases)에서 다운받을 수 있습니다.

<pre class="prettyprint">
mkdir -p ~/ros2_install
cd ~/ros2_install
tar xf ~/Downloads/ros2-bouncy-macos-x86_64.tar.bz2
</pre>

<br>

## 테스트

다음 명령어를 이용해서 잘 동작하는지 확인해봅니다. 터미널 2개를 열어서 각 터미널간 메시지 전송이 잘되는지 확인합니다.

<pre class="prettyprint">
. ~/ros2_install/ros2-osx/setup.bash

ros2 run demo_nodes_cpp talker
</pre>

<pre class="prettyprint">
. ~/ros2_install/ros2-osx/setup.bash

ros2 run demo_nodes_cpp listener
</pre>

<br>

## CSR UTIL disabled

본격적인 ROS 2.0 개발을 하기 위해서는 맥 OS의 기본 설정인 `CSR UTIL`의 상태를 `disable`로 변경해주어야 합니다.
그렇지 않으면 터미널 외 다른 IDE 등에서 ROS 2.0 환경 설정을 가져오거나 동적 라이브러리 연동이 어렵습니다.
`CSR UTIL`를 `disable`로 변경하는 방법은 다음과 같습니다.

맥 OS를 재시작하면서 <kbd>Command</kbd> + <kbd>R</kbd> 버튼을 눌러서 `리커버리 모드`로 진입합니다. 
거기서 터미널을 열고 다음 명령어를 수행합니다.

<pre class="prettyprint">
$ csrutil status

$ csrutil disable
</pre>

그 이후 재부팅을 해야 시스템에 적용됩니다.