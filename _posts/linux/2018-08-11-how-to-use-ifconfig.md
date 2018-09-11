---
layout: post
title: ifconfig 명령어 사용법
category: Linux
tag: [리눅스]
---
# ifconfig 명령어 사용법

`ifconfig` 명령어는 네트워크 인터페이스 정보를 확인하거나 설정하는 명령어입니다. 사용법은 다음과 같습니다.

<br>

## 사용법

~~~
ifconfig [interface name] [address] [option]
~~~

예를 들면 `etho0` 인터페이스를 활성화하거나 비활성화하고 싶으면 다음과 같은 명령어를 내릴 수 있습니다.

<pre class="prettyprint">
ifconfig eth0 up
ifconfig eth down
</pre>

인터페이스에 ip address를 할당하거나 subnet mask 값이나 broacast 주소 등을 바꾸고 싶은 경우는 다음과 같이 사용합니다.

<pre class="prettyprint">
ifconfig eth0 192.168.0.100 netmask 255.255.255.0 broadcast 192.168.0.255 up
</pre>

패킷 스니핑(Sniffing)을 하기 위해서는 해당 인터페이스를 `promisc` 모드로 변경해서 사용할 수 있습니다.

<pre class="prettyprint">
ifconfig eth0 promisc       # promisc 모드 설정
ifconifg eth0 -promisc      # promisc 모드 해제
</pre>

MAC 주소값을 변경하고 싶은 경우는 다음과 같습니다.

<pre class="prettyprint">
ifconfig eth0 hw ether 00:11:22:33:44:55
</pre>