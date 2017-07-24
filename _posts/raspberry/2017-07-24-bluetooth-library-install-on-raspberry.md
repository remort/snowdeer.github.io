---
layout: post
title: 블루투스 라이브러리 설치
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 블루투스]
---

## 블루투스 라이브러리 설치

라즈베리파이에서 블루투스 모듈을 사용한 프로그램을 개발할 때 다음과 같은 라이브러리들을
설치해주어야 합니다.

~~~
sudo apt-get install build-essential libbluetooth-dev
~~~

<br>

## SDP Server 활성화

그리고나서 SDP Server를 활성화해주어야 하는데, dbus-org.bluez.service 파일의 내용을
다음과 같이 편집해 주면 됩니다.

~~~
sudo nano /etc/systemd/system/dbus-org.bluez.service
~~~

nano를 이용해서 `dbus-org.bluez.service` 파일을 열고, 그 안에

~~~
ExecStart=/usr/lib/bluetooth/bluetoothd
~~~

부분을 찾아서 뒤에

~~~
ExecStart=/usr/lib/bluetooth/bluetoothd --compat
~~~

와 같이 `--compat` 옵션을 붙여줍니다.


<br>

## .bashrc 편집

그 다음 현재 라즈베리파이의 홈 폴더의 `.bashrc` 파일을 열어서

~~~
nano /home/pi/.bashrc
~~~

다음과 같은 코드를 추가합니다.

~~~
sudo chmod 777 /var/run/sdp
~~~

그리고 라즈베리파이를 재시작 해주시면 됩니다.
