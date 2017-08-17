---
layout: post
title: 소켓 옵션 설정하기(setsockopt)
category: C++
tag: [C++]
---
# setsockopt

소켓의 송수신 동작을 `setsockopt()` 함수를 이용해서 다양한 옵션으로 제어할 수 있습니다.

<pre class="prettyprint">
int setsockopt(SOCKET socket, int level, int optname, const void* optval, int optlen);
</pre>

* socket : 소켓의 번호
* level : 옵션의 종류. 보통 `SOL_SOCKET`와 `IPPROTO_TCP` 중 하나를 사용
* optname : 설정을 위한 소켓 옵션의 번호
* optval : 설정 값이 저장된 주소값.
* optlen : optval 버퍼의 크기

<br>

## 예제 

예를 들어 `SO_REUSEADDR`을 설정하려면 다음과 같이 호출합니다.

<pre class="prettyprint">
int reuseAddress = 1;
setsockopt(socket, SOL_SOCKET, (const char*)&reuseAddress, sizeof(reuseAddress));
</pre>

<br>

## SOL_SOCKET 레벨

`setsockopt()` 함수의 `level`에 들어갈 값입니다.

값 | 자료형 | 설명
--- | --- | ---
SO_REUSEADDR | BOOL | 이미 사용된 주소를 재사용하도록 함
SO_RCV_BUF | int | 수신용 버퍼의 크기 지정
SO_SND_BUF | int | 송신용 버퍼의 크기 지정
SO_RECVTIMEO | DWORD(timeval) | 수신시 Blocking 제한 시간을 설정 
SO_SNDTIMEO | DWORD(timeval) | 송신시 Blocking 제한 시간을 설정
SO_KEEPALIVE | BOOL | TCP 통신에서만 유효. 일정 시간마다 연결 유지 상태를 체크.
SO_LINGER | struct LINGER | 소켓을 닫을 때 남은 데이터의 처리 규칙 지정
SO_DONTLINGER | BOOL | 소켓을 닫을때 남은 데이터를 보내기 위해서 블럭되지 않도록 함
SO_DONTROUTE | BOOL | 라우팅(Routing)하지 않고 직접 인터페이스로 전송
SO_BROADCAST | BOOL | 브로드캐스트 사용 가능 여부

<br>

## SO_REUSEADDR 옵션

소켓 사용시 만약 다음과 같은 에러가 발생하는 경우가 있습니다. 

~~~
bind error : Address already in use
~~~

보통 소켓을 사용하는 프로그램은 강제 종료되었지만, 커널단에서 해당 소켓을 바인딩해서 사용하고 있기 때문에 발생하는 에러입니다.

이 경우 `SO_REUSEADDR` 옵션을 이용해서 기존에 바인딩된 주소를 다시 사용할 수 있게 할 수 있습니다.

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

<br>

## SO_LINGER 옵션

`linger` 구조체는 다음과 같은 형태로 되어 있습니다.
<pre class="prettyprint">
struct linger {
  int l_onoff;
  int l_linger;
}
</pre>

* l_onoff : linger 옵션의 On/Off 여부
* l_linger : 기다리는 시간

위의 두 개의 변수 값에 따라 3 가지의 close 방식이 존재합니다.

1. l_onoff == 0 : 소켓의 기본 설정 l_linger에 관계없이 버퍼에 있는 모든 데이터를 전송. `close()`는 바로 리턴을 하지만 백그라운드에서 이러한 작업이 이루어짐.

2. l_onoff > 0, l_linger == 0 : `close()`는 바로 리턴하며, 버퍼에 있는 데이터는 버림. TCP 연결 상태에서는 상대편 호스트에게 리셋을 위한 `RST` 패킷 전송. hard 혹은 abortive 종료라고 부름.

3. l_onoff > 0, l_linger > 0 : 버퍼에 남아있는 모든 데이터를 보내며, 그 동안 `close()`는 블럭되어 대기함.