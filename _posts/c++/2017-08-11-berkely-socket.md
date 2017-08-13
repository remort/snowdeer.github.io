---
layout: post
title: 버클리 소켓(Berkely Socket)에 대하여 
category: C++
tag: [C++]
---
# 버클리 소켓이란

버클리 소켓은 원래 BSD 4.2 운영체제의 일부로 배포되었다가, 그 후 여러 OS들이나 플랫폼, 프로그래밍 언어 등에 포팅되면서 네트워크 프로그래밍의 표준처럼 여겨지게 되었습니다.

<br>

## 소켓 생성

소켓 생성은 다음 명령어를 이용해서 할 수 있습니다.

<pre class="prettyprint">
SOCKET socket(int af, int type, int protocol);
</pre>

<br>

### Address Family

`af` 파라메터는 주소 패밀리(Address Family)를 의미하며, 다음과 같은 값을 가질 수 있습니다.

상수값 | 의미
------ | -----
AF_UNSPEC | 지정하지 않음
AF_INET | 인터넷 프로토콜 (IPv4)
AF_INET6 | 인터넷 프로토콜 (IPv6)
AF_IPX | IPX(Internetwork Packet Exchange) 프로토콜

<br>

### Type

`type` 파라메터는 소켓을 통해서 주고 받는 패킷의 종류를 의미합니다.

상수값 | 의미
------ | -----
SOCK_STREAM | 순서가 보장되는 데이터 스트림
SOCK_DGRAM | 데이터그램(Datagram)을 패킷을 전송
SOCK_RAW | Raw level의 패킷을 전송
SOCK_SEQPACKET | SOCK_STREAM과 비슷하나 패킷 수신 시 항상 전체를 읽어들여야 함

`SOCK_STREAM` 형태의 타입으로 지정을 하면 소켓은 상태유지형(stateful) 연결 형태가 됩니다. 신뢰성이 있고 순서가 보장되는 스트림(Stream) 형태로 데이터를 처리할 수 있습니다. TCP 프로토콜에 어울리는 소켓 형식입니다.

`SOCK_DGRAM` 타입은 UDP 프로토콜에 어울리는 소켓 형식으로 연결 상태를 유지할 필요가 없기 때문에 최소한의 리소스만 할당하여, 개별 데이터그램 단위로만 데이터를 주고 받을 수 있게 됩니다. 신뢰성이나 패킷의 순서를 보장할 필요가 없습니다.

<br>

### Protocol

`protocol` 파라메터는 소켓이 실제로 사용할 프로토콜의 종류를 명시합니다.

상수값 | 필요 소켓 타입 | 의미
------ | ----- | -----
IPPROTO_TCP | SOCK_STREAM | TCP 세그먼트 패킷
IPPROTO_UDP | SOCK_DGRAM | UDP 데이터그램 패킷
IPPROTO_IP 또는 0 | 상관없음 | 주어진 소켓 종류의 디폴트 프로토콜을 사용

`protocol`을 '0'으로 하면 운영체제가 알아서 적절한 프로토콜을 선택해주기 때문에 다음과 같이 사용해도 됩니다.

<pre class="prettyprint">
SOCKET tcpSocket = socket(AF_INET, SOCK_STREAM, 0);

SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
</pre>

<br>

## 소켓 종료 

소켓을 종료할 때는 다음 코드로 종료합니다.

<pre class="prettyprint">
int closesocket(SOCKET socket);
</pre>

만약 소켓을 닫기 전에 전송과 수신을 종료하려면

<pre class="prettyprint">
int shutdown(SOCKET socket, int how);
</pre>

함수를 호출하면 됩니다. `how` 파라메터는 다음과 같습니다.

상수값 | 의미
------ | -----
SD_SEND | 전송을 중단
SD_RECEIVE | 수신을 중단
SD_BOTH | 송수신을 모두 중단