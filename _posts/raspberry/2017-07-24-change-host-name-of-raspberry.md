---
layout: post
title: 라즈베리파이 호스팅 네임 변경
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 블루투스]
---

라즈베리파이의 호스팅 네임(Hosting Name)을 변경하는 방법입니다. 라즈베리파이의
디바이스 이름이 변경되며, 블루투스 등에서 기기를 검색할 때 뜨는 이름도 변경됩니다.

<br>

# /etc/hostname

~~~
sudo nano /etc/hostname
~~~

기본 이름으로 `raspberrypi`가 입력되어 있을 텐데, 원하는 이름으로 변경합니다.
저 같은 경우는 `snowdeer-raspberry`로 변경했습니다. 

<br>

# /etc/hosts

DNS 매핑을 관리하는 파일입니다. 이 부분을 제대로 설정하지 않으면
종종 다음과 같은 오류 메세지를 볼 수 있습니다.

~~~
sudo: unable to resolve host snowdeer-raspberry
~~~

다음 명령어를 이용해서 `hosts' 파일을 열어줍니다. 

~~~
sudo nano /etc/hosts
~~~

`127.0.0.1 raspberrypi`로 되어 있는 부분을 `127.0.0.1 snowdeer-raspberry`로
변경하시면 됩니다.
