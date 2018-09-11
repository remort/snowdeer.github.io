---
layout: post
title: ip 명령어 사용법
category: Linux
tag: [리눅스]
---
# ip 명령어 사용법

`ip` 명령어를 이용해서 ip 주소 정보 조회나 ip 설정 등을 할 수 있습니다. `ifconfig` 명령어와 사용법이 비슷합니다.

<br>

## 사용법

~~~
ip [option] [대상] [command]
~~~

<br>

## 사용 예제
<pre class="prettyprint">
ip addr show                                            # ip 정보 출력
ip addr add 192.168.5.10/24 dev eth0                    # eth0 인터페이스에 ip 설정
ip addr del 192.168.5.10/24 dev eth0                    # eth0 인터페이스의 ip 삭제
ip link set eth0 up                                     # eth0 인터페이스 활성화
ip link set eth0 down                                   # eth0 인터페이스 비활성화
ip route show                                           # 라우팅 정보 출력
ip route add default via 192.168.5.1                    # 게이트웨이 설정
ip route del default via 192.168.5.1                    # 게이트웨이 삭제
ip route add 10.20.12.0/24 via 192.168.5.1 dev eth0     # 정적 라우팅 정보 설정
ip route del 10.20.12.0/24                              # 정적 라우팅 정보 삭제
</pre>