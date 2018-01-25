---
layout: post
title: Ubuntu Server에 Desktop GUI 설치하기
category: Linux
tag: [리눅스, Ubuntu]
---

# Ubuntu Server

Ubuntu Server는 Desktop GUI가 없고, 터미널만 존재하는 모드입니다. 여기에 Desktop GUI를 설치하기는 방법입니다.

<br>

# 설치 명령어

먼저 아래 명령어를 이용해서 `apt-get`을 업데이트해줍니다.
~~~
sudo apt-get update
sudo apt-get upgrade
~~~


그리고 아래 명령어를 이용해서 `ubuntu-desktop`를 설치해줍니다.  
~~~
apt-get install ubuntu-desktop
~~~

위와 같이 설치할 경우, 파이어폭스나 오픈오피스 등의 기본적인 프로그램들도 같이 설치되는데, 대략 700MB 정도의 설치파일을 다운로드하게 됩니다. (설치후에는 더 큰 용량을 차지합니다.)

만약 기본 프로그램들은 제외하고 Ubutu Desktop 최소 설치를 진행하려면 아래와 같은 명령어를 사용하면 됩니다. 이 때는 약 170MB 정도의 설치파일을 다운로드합니다.

~~~
sudo apt-get install --no-install-recommends ubuntu-desktop
~~~

<br>

# 실행 방법

Ubuntu Desktop를 실행하는 명령어는 `startx` 입니다.

<br>

# 추가 패키지 설치

만약 최소 설치로 진행한 경우 추가적인 패키지를 설치해주도록 합시다.

## 상단 메뉴 시간 추가

~~~
sudo apt-get install indicator-datetime
~~~

## hud service not connected 오류 해결

~~~
sudo apt-get install indicator-appmenu-tools
~~~

## 볼륨조절 아이콘 추가

~~~
sudo apt-get install indicator-applet-complete
~~~

