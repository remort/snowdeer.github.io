---
layout: post
title: netstat 명령어 사용법
category: Linux
tag: [리눅스]
---
# netstat 명령어 사용법

`netstat` 명령어를 사용하면 네트워크의 연결 상태 정보를 알 수 있습니다. 네트워크 연결 정보 외에도 라우팅 테이블 정보나 네트워크 인터페이스의 상태, Masqurade 연결 상태 및 멀티캐스트 멤버 정보 등을 조회할 수 있습니다.

<br>

## 사용법

~~~
netstat [option] [address_family_option]
~~~

옵션은 다음과 같습니다.

* -a : 모든 소켓 정보 출력
* -n : 호스트명이나 포트명 대신 숫자로 표시(예를 들어 www는 80으로 출력)
* -p : 소켓에 대한 PID 및 프로그램명 출력
* -r : 라우팅 정보 출력
* -l : Listening(대기)하고 있는 포트 출력
* -i : 네트워크 인터페이스 테이블 출력
* -s : 네트워크 통계 정보 출력
* -c : 네트워크 정보를 주기적으로 계속 출력
* -t : TCP 기반 접속 목록 출력
* -u : UDP 기반 접속 목록 출력
* -g : 멀티캐스트 그룹 정보 출력

`address_family_option` 항목은 다음과 같은 값을 가질 수 있습니다.

* --protocol=프로토콜 이름 : inet, unix, ipx, ax25 등 특정 프로토콜 관련 정보 출력
* --inet, --ip : IP 주소 기반 연결 정보 출력(`--protocol=inet`과 같은 결과) 
* --unix : Unix Domain Socket 정보 출력(`--protocol=unix`과 같은 결과)

<br>

## 사용 예제

<pre class="prettyprint">
netstat -anp    # 모든 소켓의 PID 및 프로그램 정보를 출력하고 호스트명이나 포트명은 숫자로 출력
netstat -r      # 라우팅 테이블 정보 출력
</pre>