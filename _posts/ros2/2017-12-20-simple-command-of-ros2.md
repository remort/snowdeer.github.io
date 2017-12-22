---
layout: post
title: ROS2 간단한 명령어 모음
category: ROS2
tag: [ROS]
---

ROS 2.0의 실행 명령어는 `ros2` 입니다. ROS 1.0 버전에서의 명령어 사용법과 꽤 차이가 있습니다. 간단한 사용법을 알아놓는 편이 좋습니다.

# ros2 명령어 사용법

## 도움말

간단한 도움말을 보기 위해서는 `--help` 옵션을 넣으면 됩니다.

~~~
$ ros2 --help
~~~

<br>

## 프로그램 실행방법

`run` 옵션을 이용해서 프로그램을 실행할 수 있습니다. 하지만 경험상 그냥 실행 파일만 실행해도 잘 실행이 되더군요.

~~~
ros2 run demo_nodes_cpp talker -t chatter2
~~~

<br>

## Message Publish

특정 Topic으로 메시지를 발행하기 위해서는 `topic pub` 옵션을 사용하면 됩니다. 아래 예제는 `snowdeer_topic`이라는 이름의 Topic에 문자열 메시지를 보내는 예제입니다.

~~~
$ ros2 topic pub /snowdeer_topic std_msgs/String "data: Hello, snowdeer."
~~~

<br>

## Message Subscribe

특정 Topic의 메시지를 수신하기 위해서는 `topic echo` 옵션을 사용하면 됩니다.

~~~
$ ros2 topic echo /snowdeer_topic
~~~

<br>

## Topic 리스트 조회

~~~
$ ros2 topic list
~~~

<br>

## Node 리스트 조회

~~~
$ ros2 node list
~~~