---
layout: post
title: 소켓 옵션 설정하기(setsockopt)
category: C++
tag: [C++]
---
# setsockopt

소켓의 송수신 동작을 `setsockopt()` 함수를 이용해서 다양한 옵션으로 제어할 수 있습니다.

<pre class="prettyprint">
int setsockopt(SOCKET socket, int level, int optname, const char* optval, int optlen);
</pre>

`level`과 `optname`을 쌍으로 해서 옵션의 종류를 지정합니다. `level`은 옵션의 범주를 나타내는 정수값이며, `optname`은 개별 옵션을 뜻합니다.

`optval`은 옵션 값이 담겨진 포인터 주소값입니다. 항상 `const char*`로 캐스팅해서 넘겨야 합니다. 예를 들어 `SO_REUSEADDR`을 설정하려면 다음과 같이 호출해야 합니다.

<pre class="prettyprint">
int reuseAddress = 1;
setsockopt(socket, SOL_SOCKET, (const char*)&reuseAddress, sizeof(reuseAddress));
</pre>

<br>

## SOL_SOCKET 레벨

`setsockopt()` 함수의 `level`에 들어갈 값입니다.

값 | 자료형 | 설명
--- | --- | ---
SO_RCV_BUF | int | 해당 소켓이 수신용으로 사용할 버퍼의 크기를 지정. 수신된 데이터는 프로세스가 `recv()`나 `recvfrom()`을 호출할 때까지 버퍼에 쌓여있게 됨. TCP 대역폭은 윈도우의 크기에 좌우되며, 소켓의 수신 버퍼 크기보다 커질 수 없기 때문에 이 값을 변경하면 대역폭에 큰 영향을 미치게 됨.
SO_REUSEADDR | BOOL(int) | 네트워크 계층이 다른 소켓에 이미 할당된 IP 주소와 포트가 있을 때 중복해서 바인딩하는 것을 허용할지 결정하는 옵션. 관리자 권한이 있어야 이 옵션이 실행되는 운영체제도 있음.
SO_RECVTIMEO | DWORD(timeval) | 수신 동작의 Blocking 제한 시간을 설정. Windows에서는 msec 단위임.
SO_SND_BUF | int | 송신용으로 쓸 버퍼의 크기를 지정. 송신 대역폭은 링크 계층에 좌우됨. 프로세스에서 링크 계층의 한도 이상으로 데이터를 보내고자 할 때 나머지 데이터를 송신 버퍼에 저장함. TCP 통신에서는 소켓이 송신한 데이터의 확인 응답을 받을 때까지 재전송용으로 패킷을 쌓아두는데 이 때 이 버퍼를 사용함. 전송 버퍼가 가득찰 경우 자리가 생길 때까지 `send()`나 `sendto()` 호출이 Blocking 됨.
SO_SNDTIMEO | DWORD(timeval) | 송신 동작의 Blocking 제한 시간을 설정.
SO_KEEPALIVE | BOOL(int) | TCP 통신에서만 유효. 일정 시간마다 연결 유지 상태를 체크함.

<br>

## TCP_NODELAY 옵션

IPPROTO_TCP 레벨의 옵션이며, TCP 소켓에서만 사용할 수 있습니다.


값 | 자료형 | 설명
--- | --- | ---
TCP_NODELAY | BOOL(int) | 소켓에 [네이글 알고리즘(Nagle Algorithm)](https://en.wikipedia.org/wiki/Nagle%27s_algorithm)을 사용할지 여부를 지정. **`TRUE`인 경우 알고리즘을 사용하지 않음.** 알고리즘을 사용하지 않을 경우 실제 전송까지의 지연 시간이 줄어듬.

값이 `1` 또는 `TRUE`인 경우 네이글 알고리즘을 사용하지 않습니다.
값이 `2` 또는 `FALSE`인 경우 네이글 알고리즘을 사용합니다.

예제 코드는 다음과 같습니다.

<pre class="prettyprint">
int opt_val = TRUE;
setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt_val, sizeof(opt_val));
</pre>