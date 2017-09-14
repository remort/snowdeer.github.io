---
layout: post
title: 네트워크(WiFI) 설정 방법
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
일반적으로 라즈베리파이는 GUI 환경이 있어서 GUI 상에서 설정을 하면 편리하지만, 나노파이(NanoPI) 또는 라즈베리파이의 non-GUI 버전을 사용할 경우에는 터미널 상에서 네트워크 세팅을 해주어야 합니다.

다음과 같이 파일을 설정하면 됩니다.

# Interface 수정

`/etc/network/interfaces` 파일을 열어서 다음과 같이 수정해줍니다. 파일이 없다면 생성하시면 됩니다.
~~~
auto lo
iface lo inet loopback

auto wlan0

allow-hotplug wlan0
iface wlan0 inet dhcp
wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
iface default inet dhcp
~~~

<br>

# SSID 정보 입력

`/etc/wpa_supplicant/wpa_supplicant.conf` 파일을 다음과 같이 수정해줍니다.

~~~
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev

update_config=1

network={
        ssid="snowdeer_AP"
        scan_ssid=1
        psk="password"
        key_mgmt=WPA-PSK
}
~~~

여기서 `scan_ssid` 항목은 공유기의 속성이 `숨김`으로 되어 있을 때 사용하는 옵션입니다.

설정 이후 재부팅을 하면 라즈베리파이는 자동으로 공유기에 접속되며 IP를 할당받습니다.