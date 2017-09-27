---
layout: post
title: VNC를 통한 라즈베리 원격 제어
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# VNC를 통한 라즈베리 원격 제어

VNC(Virtual Network Connection)를 이용하면 라즈베리파이를 원격에서 GUI 환경에서 쉽게 제어할 수 있습니다.

<br>

## VNC 서버 설치

VNC(Virtual Network Connection) 서버는 다음 명령어를 이용해서 설치할 수 있습니다.

~~~
$ sudo apt-get update
$ sudo apt-get install tightvncserver
~~~

<br>

## VNC 서버 실행

VNC 서버를 설치한 후, 다음 명령어를 이용해서 VNC 서버를 실행할 수 있습니다.

~~~
$ vncserver :1
~~~

<br>

## VNC 서버에 접속

원격에서는 VNC 클라이언트를 설치해야 합니다. 다양한 VNC 프로그램들이 있으며, [RealVNC](https://www.realvnc.com/en/) 등을 사용할 수 있습니다. 

접속할 때 IP 주소 뒤에 `:1`을 입력하면 화면 번호 '1'에 접속할 수 있습니다.

<br>

## 라즈베리파이 시작할 때 VNC 서버 자동 실행하기

다음 명령어를 이용해서 라즈베리파이를 시작할 때 VNC 서버를 자동으로 실행되도록 할 수 있습니다.

~~~
$ cd /home/pi
$ cd .config
$ mkdir autostart
$ cd autostart
$ nano tightvnc.desktop
~~~

그 후 `tightvnc.desktop` 내용에 다음을 입력합니다.

~~~
[Desktop Entry]
Type=Application
Name=TightVNC
Exec=vncserver :1
StartupNotify=false
~~~