---
layout: post
title: MacOS에 ROS 2.0 설치하기(Eloquent Elusor)
category: ROS2
tag: [ROS]
---

# 기본 요소 설치

<pre class="prettyprint">
brew install python3
brew install asio tinyxml2
brew install tinyxml eigen pcre poco

brew install qt freetype assimp
brew install console_bridge
brew install log4cxx
brew install cunit

brew install graphviz

python3 -m pip install pygraphviz pydot
python3 -m pip install lxml
python3 -m pip install catkin_pkg cryptography empy ifcfg lark-parser lxml netifaces numpy pyparsing pyyaml setuptools argcomplete
</pre>

<br>

# ROS2 바이너리 다운로드

ROS 2.0 바이너리는 [여기](https://github.com/ros2/ros2/releases)에서 다운로드 가능합니다.

<pre class="prettyprint">
mkdir -p ~/ros2_eloquent
cd ~/ros2_eloquent

tar xf ros2-eloquent-20200124-macos-amd64.tar.bz2
</pre>

<br>

# 설치 확인

<pre class="prettyprint">
. ~/ros2_eloquent/ros2-osx/local_setup.zsh
ros2 run demo_nodes_cpp talker
</pre>

<pre class="prettyprint">
. ~/ros2_eloquent/ros2-osx/local_setup.zsh
ros2 run demo_nodes_py listener
</pre>

<br>

# 에러 발생시

### Poco 버전 관련 에러

현재 MacOS(Catalina 버전 이후)에서 ROS 2.0 바이너리를 설치한 후 `talker`, `listener`와 같은 예제 명령어를 실행하면 아래와 같은 오류 메시지가 나오면서 실행이 되지 않습니다. `Poco` 라이브러리 버전 때문에 발생하는 문제입니다.

~~~
Failed to load entry point 'launch': dlopen(/Users/justinmarple/ros2_eloquent/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: /usr/local/opt/poco/lib/libPocoFoundation.63.dylib
  Referenced from: /Users/justinmarple/ros2_eloquent/ros2-osx/lib/librosidl_typesupport_c.dylib
  Reason: image not found
The C extension '/Users/justinmarple/ros2_eloquent/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so' failed to be imported while being present on the system. Please refer to 'https://index.ros.org/doc/ros2/Troubleshooting/#import-failing-even-with-library-present-on-the-system' for possible solutions
Failed to load entry point 'info': dlopen(/Users/justinmarple/ros2_eloquent/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: /usr/local/opt/poco/lib/libPocoFoundation.63.dylib
  Referenced from: /Users/justinmarple/ros2_eloquent/ros2-osx/lib/librosidl_typesupport_c.dylib
  Reason: image not found
~~~

이 경우는 ROS 2.0 을 소스로 다운받아서 빌드 후 설치하거나 아래 명령어로 임시로 실행되도록 헐 수 있습니다.

<pre class="prettyprint">
ln -s /usr/local/opt/poco/lib/libPocoFoundation.71.dylib /usr/local/opt/poco/lib/libPocoFoundation.63.dylib
</pre>

sudo apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib build-essential chrpath socat libsdl1.2-dev xterm

<br>

### TinyXml2 버전 관련 에러

~~~
dyld: Library not loaded: /usr/local/opt/tinyxml2/lib/libtinyxml2.7.dylib
  Referenced from: /Users/snowdeer/ros2_eloquent/ros2-osx/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.dylib
  Reason: image not found
~~~

만약 위와 같은 오류가 발생한다면 먼저 tinyxml2 라이브러리 버전을 최신으로 설치합니다.

<pre class="prettyprint">
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)" < /dev/null 2> /dev/null

brew install tinyxml2
</pre>

이 글을 포스팅하는 현 시점에는 `tinyxml2 8.0`이 설치가 되네요. ROS2는 7.0 이후부터 지원합니다. 하지만 바이너리 빌드시 7.0으로 링크가 되어 있기 때문에 8.0을 7.0 버전으로 사용할 수 있도록 다음 명령어를 추가로 실행합니다.

<pre class="prettyprint">
cd /usr/local/opt/tinyxml2/lib
cp libtinyxml2.8.dylib libtinyxml2.7.dylib
</pre>