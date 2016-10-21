---
layout: post
title: ADB 무선 연결
category: Android
tag: [android, adb]
---

안드로이드 개발할 때, ADB(Android Debug Bridge) 연결은 필수입니다.
보통 USB 케이블을 이용해서 연결하면 되는데, 만약 케이블 연결이 
불편한 상황이라면? 

~~~
ex. 제 노트북은 USB 포트가 USB-C type 1개 뿐이라, 
충전 케이블을 꽂으면 핸드폰을 노트북에 연결할 수가 없습니다. ㅜ_ㅜ;
~~~

WiFi를 이용해서 ADB 연결을 시도하면 됩니다 !!  
(물론, 한 번은 케이블을 이용해서 설정을 해줘야 합니다.)

<br>

## 연결 절차

* 노트북과 스마트폰을 동일 공유기로 접속시켜줍니다.
* 스마트폰을 USB 케이블을 이용해 노트북에 연결합니다.
* cmd 명령어를 통해 다음과 같이 실행합니다.

~~~
adb tcpip 5555
restarting in TCP mode port: 5555
~~~

* 그리고 스마트폰의 IP 주소를 얻기 위해서 다음과 같이 실행합니다.

~~~
adb shell ifconfig
~~~

* 이제 설정이 끝났습니다. USB를 해제해도 됩니다. 
* PC에서 스마트폰에 연결하기 위해서는 다음과 같이 입력하면 됩니다.

~~~
adb connect [IP Address]:5555
ex) adb connect 192.168.0.5:5555
connected to 192.168.0.140:5555
~~~

* 현재 접속이 잘 되어 있는지 확인하려면, cmd에서 다음과 같이 입력하면 됩니다.

~~~
C:\>adb devices
List of devices attached
192.168.0.140:5555      device
~~~