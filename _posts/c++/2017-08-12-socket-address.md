---
layout: post
title: 소켓 주소
category: C++
tag: [C++, 네트워크]
---

모든 패킷에는 보내는 주소와 받는 주소가 필요합니다. 특히, 전송 계층의 패킷은 추가로 발신지와 수신지의 포트도 필요합니다. 이러한 정보는 `sockaddr` 구조체에 정의되어 있습니다.

# sockaddr 구조체

## MacOS 용 sockaddr

<pre class="prettyprint">
struct sockaddr {
  __uint8_t sa_len;        /* total length */
  sa_family_t sa_family;    /* [XSI] address family */
  char sa_data[14];    /* [XSI] addr value (actually larger) */
};
</pre>

## Cygwin 용 sockaddr

<pre class="prettyprint">
struct sockaddr {
  sa_family_t sa_family;    /* address family, AF_xxx	*/
  char sa_data[14];    /* 14 bytes of protocol address	*/
};
</pre>

`sa_family`는 주소의 종류를 나타내는 상수값입니다. `sa_data` 필드에 바이트(Byte) 형태로 주소값이 저장됩니다. 

`sa_data`에 들어갈 값은 `sa_family` 포맷에 따라 다양한 형태로 저장될 수 있기 때문에, 좀 더 편리하게 작성한 다음 캐스팅(Casting)을 통해 데이터를 변환하는 것을 추천합니다. 

IPv4 패킷용 주소를 만들기 위해서는 `sockaddr_in` 자료형을 사용할 수 있습니다.

<br>

# sockaddr_in 구조체

## MacOS 용 sockaddr_in

<pre class="prettyprint">
struct sockaddr_in {
  __uint8_t sin_len;
  sa_family_t sin_family;
  in_port_t sin_port;
  struct in_addr sin_addr;
  char sin_zero[8];
};
</pre>

## Cygwin용 sockaddr_in

<pre class="prettyprint">
struct sockaddr_in {
  sa_family_t sin_family;    /* Address family		*/
  in_port_t sin_port;    /* Port number			*/
  struct in_addr sin_addr;    /* Internet address		*/

  /* Pad to size of `struct sockaddr'. */
  unsigned char __pad[__SOCK_SIZE__ - sizeof(short int)
      - sizeof(unsigned short int) - sizeof(struct in_addr)];
};
</pre>

`sin_family`는 메모리 레이아웃상 `sockaddr`의 `sa_family`와 같은 기능을 합니다.

`sin_addr`는 4바이트의 IPv4 주소입니다. 소켓 라이브러리마다 조금씩 다르며, 어떤 플랫폼에서는 단순히 4바이트 정수이기도 합니다. 이 때문에 여러 플랫폼에서 여러 구조체를 유니온으로 감싸둔 구조체로 값을 지정할 수 있게 해 놓았습니다.

`sin_len`은 플랫폼에 따라 존재하기도 하는 변수인데 구조체의 길이를 지정하는 변수입니다. 다음과 같은 형태로 사용하면 됩니다.

<pre class="prettyprint">
addr.sin_len = sizeof(addr);
</pre>

# 바이트 주소 체계 변환

IP 주소를 바이트 형태로 묶어서 사용할 때 서버/클라이언트간 바이트 순서 체계가 다를 수 있습니다. 이런 문제를 방지하게 위해서 바이트 값들은 네트워크 순서 쳬계로 변경해서 사용해야 합니다.

<pre class="prettyprint">
uint16_t htons(uint16_t hostshort);
uint32_t htonl(uint32_t hostlong);
</pre>

`htons()`는 부호 없는 16비트 정수를 받아서 네트워크 바이트 순서로 변환합니다. `htonl()`은 32비트 정수에 대해서 변환을 수행합니다.

반대의 경우는 

<pre class="prettyprint">
uint16_t ntohs(uint16_t networkshort);
uint32_t ntohl(uint32_t networklong);
</pre>

<br>

# 예제 코드

다음과 같은 코드 형태로 사용할 수 있습니다.

## Server Address

<pre class="prettyprint">
  struct sockaddr_in server_address;
  memset(server_address.sin_zero, 0, sizeof(server_address.sin_zero));
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons(80);
  server_address.sin_len = sizeof(server_address);
</pre>

여기서 `INADDR_ANY`는 서버로 들어오는 모든 데이터를 송수신하기위한 상수값입니다.


<br>

## Client Address

<pre class="prettyprint">
  struct sockaddr_in client_address;
  memset(client_address.sin_zero, 0, sizeof(client_address.sin_zero));
  client_address.sin_family = AF_INET;
  client_address.sin_port = htons(80);
  client_address.sin_addr.s_addr = inet_addr("192.168.0.10");
  client_address.sin_len = sizeof(client_address);
</pre>

`inet_addr()` 함수의 경우, `htonl()`을 사용할 필요 없이 문자열의 값을 `long` 값으로 변환해주는 함수입니다.