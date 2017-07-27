---
layout: post
title: Wi-FI를 이용한 ADB 연결
category: Android
tag: [Android, ADB]
---

안드로이드 개발을 할 때 ADB 연결은 필수입니다. 보통은 케이블을 이용하면 되는데,
케이블 연결이 번거로운 경우에는 케이블 연결없이 Wi-Fi 기반으로 ADB 연결을 할 수 있습니다.

> 이런 경우가 얼마냐 있을까 싶지만, USB 연결 포트가 1개밖에 없는 노트북이 종종 있습니다.
대표적으로 제가 쓰고 있는 '갤럭시탭 프로 S'를 들 수 있습니다. USB-C 포트가 1개뿐이라
충전 케이블을 꽂으면 다른 케이블을 꽂을 수 없습니다. 그렇다고 USB 허브를 매번 들고
다닐 수도 없는 일이라 개발용으로 쓰기엔 난감합니다.

물론, Wi-Fi 기반으로 연결하긴 하지만 최초에 한 번은 케이블을 이용해서 연결을 해줘야 합니다.

<br>

# 설정 방법

<ul>
 	<li>먼저 노트북과 스마트폰을 동일 공유기에 접속시켜 줍니다.</li>
 	<li>스마트폰을 USB 케이블을 이용해서 노트북에 연결합니다.</li>
 	<li>cmd 명령어를 통해서 다음과 같이 실행합니다.</li>
</ul>

~~~
adb tcpip 5555
restarting in TCP mode port: 5555
~~~

<br>

<ul>
 	<li>그리고 다음 명령어를 통해서 스마트폰의 IP Address를 알아냅니다.</li>
</ul>

~~~
adb shell ifconfig
~~~

<ul>
 	<li>이제 끝났습니다. 케이블 연결을 해제해도 됩니다.</li>
 	<li>PC에서 스마트폰에 ADB 연결을 하기 위해서는 다음 명령어를 입력하면 됩니다.</li>
</ul>

~~~
adb connect [IP Address]:5555
ex) adb connect 192.168.0.140:5555
~~~

<ul>
 	<li>현재 ADB 연결이 잘 되어 있는지 확인하기 위해서는 다음 명령어를 입력하면 됩니다.</li>
</ul>

~~~
C:>adb devices
List of devices attached
192.168.0.140:5555 device
~~~
