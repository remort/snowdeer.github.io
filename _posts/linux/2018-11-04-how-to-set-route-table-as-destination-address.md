---
layout: post
title: IP 목적지에 따라 다른 네트워크 인터페이스 사용하는 방법

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# IP 목적지별로 다른 route table 사용하는 방법

두 개(내부망/외부망)의 네트워크 인터페이스를 사용할 경우 다음과 같은 명령어를 이용해서 목적지 주소에 따라 routing을 다르게 할 수 있습니다.

~~~
# 설정: Destination IP Address가 10.XXX.XXX.XX 인 경우, en7 인터페이스 사용
sudo route -nv add -net 10 -interface en7
sudo route -nv add -net 112 -interface en7

# 삭제
sudo route delete -net 10 -interface en7
~~~