---
layout: post
title: NAT와 NAPT
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어, 네트워크]
---
기업이나 가정 등의 폐쇄된 네트워크에서 사용하는 프라이빗 IP 주소를 인터넷의 글로벌 IP 주소로 변환해주는 기술에는 NAT(Network Address Translation)와 NAPT(Network Address Port Translation)이 있습니다. 

NAT와 NAPT의 처리는 인터넷과 내부 LAN이 연결되는 라우터(Router)나 방화벽에서 수행합니다.

<br>

# NAT

NAT는 프라이빗 IP 주소와 글로벌 IP 주소를 1:1로 연결합니다. 

NAT는 LAN에서 인터넷으로 연결할 때에는 출발지 IP 주소를 변환합니다. 반대로 인터넷에서 LAN으로 연결할 때는 목적지 IP 주소를 변환합니다.

<br>

# NAPT

NAPT는 프라이빗 IP 주소와 글로벌 IP 주소를 N:1로 연결합니다. 

NAPT는 LAN에서 인터넷으로 접근할 때 출발지 IP 주소와 출발지 포트를 같이 변환합니다.

Linux에서는 NAPT를 IP masquerade라고 부릅니다.

<br>

![image](/assets/common-sense/001.png)