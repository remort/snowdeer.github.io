---
layout: post
title: 라즈베리 파이 설정 쉽게 변경하기 (raspi-config)
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# Configuration Tool 다운로드

라즈베리파이에서는 `raspi-config`라는 프로그램을, 나노파이에서는 `npi-config`라는 프로그램을 이용해서 단말기의 설정을 쉽게 변경할 수 있습니다.

![Image](/assets/2017-09-02-raspberry-raspi-config/01.png)

`raspi-config` 프로그램은 다음과 같은 기능을 제공합니다.

* `pi` 계정의 패스워드 변경
* 네트워크 호스트 이름(Hostname) 변경
* 부팅 옵션 변경
* 언어 및 지역 설정
* GPIO를 포함한 인터페이스(Interface) 설정
* 오버클럭(Overclock)
* 업데이트
* 기타 설정

<br>

## raspi-config 설치 및 실행

~~~
$ sudo apt-get install raspi-config

$ sudo raspi-config
~~~

<br>

## npi-config 설치 및 실행

~~~
$ sudo apt-get install npi-config

$ sudo npi-config
~~~

