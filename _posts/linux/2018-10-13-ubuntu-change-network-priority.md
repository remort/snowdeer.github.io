---
layout: post
title: Network Interface 우선 순위 변경하기
category: Linux
tag: [리눅스]
---
# Network Interface 우선 순위 변경하기

Ubuntu에서 랜카드를 여러 개 꽂아서 네트워크 인터페이스(Network Interface)가 여러 개 존재할 경우 우선 순위를 바꾸는 방법입니다.

먼저 `ifmetric`를 설치합니다.

<pre class="prettyprint">
sudo apt install ifmetric
</pre>

이후 다음 명령어를 이용해 Routing Table을 확인합니다.

<pre class="prettyprint">
$ route -n

Kernel IP routing table
Destination     Gateway         Genmask     Flags   Metric  Ref     Use     Iface
0.0.0.0         10.51.0.1       0.0.0.0     UG      100     0       0       eth0
0.0.0.0         192.168.0.1     0.0.0.0     UG      600     0       0       wlan0
</pre>

맨 뒤의 `Iface` 항목이 각 네트워크 인터페이스 이름이며 `Metric` 항목이 우선 순위라고 생각하면 됩니다. `Metric` 값이 낮을 수록 우선 순위가 높습니다.

`ifmetric` 명령어를 이용해서 다음과 같이 우선 순위를 변경할 수 있습니다.

<pre class="prettyprint">
sudo ifmetric wlan0 50
</pre>

다시 `route -n` 명령어로 Routing Table을 확인해봅니다.

<pre class="prettyprint">
$ route -n

Kernel IP routing table
Destination     Gateway         Genmask     Flags   Metric  Ref     Use     Iface
0.0.0.0         192.168.0.1     0.0.0.0     UG      50      0       0       wlan0
0.0.0.0         10.51.0.1       0.0.0.0     UG      100     0       0       eth0
</pre>

우선 순위가 바뀐 것을 확인할 수 있습니다.