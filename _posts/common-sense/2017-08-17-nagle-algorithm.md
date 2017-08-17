---
layout: post
title: 네이글 알고리즘(Nagle Algorithm)
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어]
---
# 일반적인 TCP 통신 방법

TCP 통신은 상대방이 패킷을 받았는지 안 받았는지 확인하기 위해서, 데이터를 받은 쪽에서 `ACK` 신호를 보냅니다. `ACK` 신호를 확인해야 송신부에서는 패킷 전송이 제대로 되었음을 확인하고 그 다음 패킷을 계속해서 전송할 수 있습니다.

<br>

# Nagle Algorithm

네이글 알고리즘은 '가능하면 조금씩 여러 번 보내지 말고, 한 번에 많이 보내라'는 원칙을 기반으로 만들어진 알고리즘입니다.

기존에는 'NAGLE'라는 단어를 패킷으로 보낸다고 할 때, 'N'을 전송한 다음 상대방으로부터 `ACK`를 받으면 'A'를 전송. 또 `ACK'를 받은 다음 'G'를 전송하는 형태로 이루어집니다. 이런식으로 반복해서 한 글자씩 차례대로 전송하게 됩니다.

하지만, 네이글 알고리즘을 적용하게 되면 처음에 'N'을 전송한다음 상대방으로부터 `ACK`를 받기 전까지 'AGLE'에 대한 패킷 정보를 송신 버퍼에 저장한다음 `ACK`가 오면 여러 개의 패킷을 모아서 한 번에 전송하게 됩니다.

![image](/assets/2017-08-17-nagle-algorithm/01.png)

<br>

# 네이글 알고리즘의 장단점

네이글 알고리즘의 장단점은 다음과 같습니다.

* 장점 : 같은 양의 데이터더라도 한 번에 많이 보내기 때문에 데이터 전송 횟수가 줄어들기 때문에 네트워크의 효율성이 높아짐
* 단점 : `ACK`를 받을 때까지 패킷을 모으고 있기 때문에 반응 속도가 느려짐

<br>

# C++ 코드 예제

C++에서 TCP 소켓 통신은 기본적으로 네이글 알고리즘이 적용되어 있습니다. `setsockopt()` 함수를 이용해서 네이글 알고리즘을 On/Off 할 수 있습니다.

다음 예제는 네이글 알고리즘을 'Off'하는 예제입니다. (값이 True이면 'Off'라는 것을 주의합시다.)

<pre class="prettyprint">
int opt_val = TRUE;
setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt_val, sizeof(opt_val));
</pre>

* 값이 `TRUE` 또는 `1` 이면 Off
* 값이 `FALSE` 또는 `2` 이면 On


