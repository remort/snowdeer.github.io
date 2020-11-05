---
layout: post
title: MacOS에 ROS 2.0 설치하기(Foxy Fitzroy)
category: ROS2
tag: [ROS]
---

# 기본 요소 설치

ROS2 Foxy 부터는 Python 3.8을 이용해야 합니다.
<pre class="prettyprint">
brew install python@3.8
brew unlink python
brew link --force python@3.8

brew install asio tinyxml2
brew install tinyxml eigen pcre poco

# OpenCV는 필수는 아닙니다. 설치시 시간이 엄청 오래 걸리니 고민해보세요.
brew install opencv

brew install openssl
echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.zshrc

brew install qt freetype assimp
brew install sip pyqt5

brew install console_bridge
brew install log4cxx spdlog
brew install cunit
brew install graphviz

python3 -m pip install pygraphviz pydot
python3 -m pip install lxml
python3 -m pip install catkin_pkg empy ifcfg lark-parser lxml netifaces numpy pyparsing pyyaml setuptools argcomplete

pip3 install -U colcon-common-extensions
</pre>

<br>

# ROS2 바이너리 다운로드

ROS 2.0 바이너리는 [여기](https://github.com/ros2/ros2/releases)에서 다운로드 가능합니다.

<pre class="prettyprint">
mkdir -p ~/ros2_foxy
cd ~/ros2_foxy
tar xf ~/Downloads/ros2-foxy-20200807-macos-amd64.tar.bz2
</pre>

<br>

# 설치 확인

<pre class="prettyprint">
. ~/ros2_foxy/ros2-osx/local_setup.zsh
ros2 run demo_nodes_cpp talker
</pre>

<pre class="prettyprint">
. ~/ros2_foxy/ros2-osx/local_setup.zsh
ros2 run demo_nodes_py listener
</pre>