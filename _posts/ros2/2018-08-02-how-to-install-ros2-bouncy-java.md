---
layout: post
title: ROS 2.0 Bouncy Java 설치 방법
category: ROS2
tag: [ROS]
---

ROS 2.0 Java 설치 방법입니다. 기존의 `Ardent` 버전에 비해서 크게 변경된 점은 없습니다.
공식 홈페이지는 [여기](https://github.com/esteve/ros2_java)입니다.

여기서는 Ubuntu 18.04 기준으로 설명을 적지만, Ubuntu 16.04에서도 동일하게 동작합니다.

<br>

## JDK 설치

<pre class="prettyprint">
sudo apt-add-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install oracle-java8-installer
</pre>

<br>

## gradle 3.2 이상 버전으로 업그레이드

<pre class="prettyprint">
sudo add-apt-repository ppa:cwchien/gradle
sudo apt install -y gradle
</pre>

<br>

## ROS2 for Java 설치 

<pre class="prettyprint">
mkdir -p ~/ros2_java_ws/src
cd ~/ros2_java_ws
curl -skL https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_desktop.repos -o ros2_java_desktop.repos
vcs import src < ros2_java_desktop.repos
. ../ament_ws/install_isolated/local_setup.sh
ament build --symlink-install --isolated
</pre>

