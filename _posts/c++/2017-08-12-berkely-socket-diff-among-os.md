---
layout: post
title: 운영체제별 버클리 소켓 차이점 
category: C++
tag: [C++]
---

버클리 소켓이 마치 표준 네트워크 API 처럼 널리 사용되고는 있지만 운영체제(OS, Operating System)별로 완전히 같지는 않습니다. OS별로 조금씩 차이가 있기 때문에 여러 플랫폼을 대상으로 개발을 할 때는 그 차이 점을 미리 알고 있는게 좋을 것 같습니다.

<br>

# 소켓 자료형의 차이

버클리 소켓의 데이터 자료형은 `SOCKET`입니다. 하지만 이 자료형을 정상적으로 갖는 OS는 Windows 10 등의 윈도우 계열에서만 가집니다. Linux, MacOS 등의 POSIX 기반의 운영체제에서는 `SOCKET`은 단순히 `int` 값을 의미하고 있습니다.

`SOCKET` 타입을 구조체로 하거나 `int`로 하는 것은 사용법에 큰 차이는 없지만 아주 약간씩의 장단점이 존재합니다. 

* `int` 값으로 사용하는 경우는 다양한 플랫폼 포팅이 좀 더 쉬움
* 대신 함수의 인자형이 `int`이기 때문에 개발자가 실수 또는 고의로 아무 값이나 집어넣어도 컴파일러(Compiler)가 전혀 알아차리지 못함

<br>

# 라이브러리 헤더 파일의 차이

윈도우용 소켓은 `WinSock2.h` 파일을 #include 해서 사용해야 합니다. (과거 `Winsock`이라는 오래된 라이브러리도 있는데, 최적화나 기능면에서 부족한 점이 많아서 추천하지 않습니다. `Winsock`은 `Windows.h` 헤더 파일에 자동으로 포함되어 있습니다.)

## Windows에서 헤더 파일간 충돌 문제

`WinSock2.h`와 `Windows.h` 헤더 파일을 동시에 #include 하는 경우 함수의 이름이 서로 같아 충돌하는 현상이 있습니다. 이를 해결하기 위해서는

* `Windows.h` 보다 `WinSock2.h` 파일을 먼저 #include 하거나,
* `WIN32_LEAN_AND_MEAN` 매크로를 #define 해야 합니다.

<br>

POSIX 계열에서는 `sys/socket.h` 헤더 파일만 #include 하면 됩니다. 그 외 다음 헤더 파일들을 추가로 포함시켜야 합니다.

헤더 파일 | 설명
--- | ---
netinet/in.h | IPv4 전용 기능을 사용할 경우
arpa/inet.h | 주소 변환 기능을 사용할 경우
netdb.h | Name Resolution을 사용할 경우

<br>

# 소켓 라이브러리 초기화/활성화 방법의 차이

POSIX에서는 소켓 라이브러리가 항상 활성화 상태로 되어 있어서, 소켓을 사용할 때 별도로 해줘야 하는 작업은 없습니다. Windows 계열에서는 초기화와 해제를 해주어야 합니다. 초기화는 `WSAStartup()` 함수를 호출합니다.

## Windows에서의 초기화 및 해제 방법

<pre class="prettyprint">
int WSAStartup(WORD wVersionRequested, LPWSADATA lpWSAData);
</pre>

`mVersionRequested`는 하위 바이트는 주 버전 번호, 상위 바이트는 부 버전 번호를 의미합니다. 보통 `MAKEWORD(2, 2)`를 인자로 입력하면 됩니다

`lpWSAData`는 WSAStrtup()이 활성화하면서 라이브러리에 대한 정보로 값을 채워주기 때문에 빈 값으로 입력해도 상관없습니다.

`WSAStartup()` 함수는 성공하면 `0`을 리턴하며 실패시 에러 코드를 리턴합니다.

라이브러리의 해제는 `WSACleanup()` 함수를 호출하면 됩니다. `WSACleanup()` 함수는 레퍼런스 카운트를 사용하므로, `WSAStartup()` 함수가 호출된 횟수만큼 `WSACleanup()` 함수도 호출을 해주어야 합니다.

<pre class="prettyprint">
int WSACleanup();
</pre>

해제시, 현재 진행 중인 모든 소켓의 동작이 강제 종료되며, 리소스는 모두 소멸이 됩니다. 따라서 라이브러리 해제 전에 모든 소켓의 마무리 작업을 먼저 진행하는 편이 좋습니다.

<br>

# 에러 코드 확인

Windows 계열에서는 에러 값을 `WSAGetLastError()` 함수를 이용해서 조회를 합니다.

<pre class="prettyprint">
int WSAGetLastError();
</pre>

반면 POSIX 계열에서는 전역 `error` 변수를 사용하기 때문에 `error.h` 헤더 파일을 #include 해서 `error` 변수 값을 확인하면 됩니다.