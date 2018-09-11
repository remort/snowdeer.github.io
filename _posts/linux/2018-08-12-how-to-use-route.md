---
layout: post
title: route 명령어 사용법
category: Linux
tag: [리눅스]
---
# route 명령어 사용법

`route` 명령어는 라우팅 테이블 정보를 조회하거나 관리할 수 있는 명령어입니다.

<br>

## 사용법

~~~
route [add|del] [-destination] [netmask] [gateway] [dev interface]
~~~

단순히 `route` 만 실행할 경우에는 기본 라우팅 테이블 정보를 출력하며, `netstat -r` 명령어와 동일한 기능을 합니다.

<br>

## 사용 예제 

`route` 명령어 사용 예제는 다음과 같습니다. 인터페이스가 하나인 경우는 뒷 부분의 `dev eth0`는 생략 가능합니다.

<pre class="prettyprint">
route add -net 192.168.5.101 netmask 255.255.255.0 dev eth0     # 네트워트 주소 설정
route del -net 192.168.5.51 netmask 255.255.255.0               # 설정된 네트워크 주소 삭제
route add default gw 192.168.5.1 eth0                           # 게이트웨이 등록
route del default gw 192.168.5.1 eth0                           # 게이트웨이 삭제
</pre>
