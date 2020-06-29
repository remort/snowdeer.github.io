---
layout: post
title: SSH 터널링
category: Tips
tag: [IDE, sublime]
---
# SSH 터널링 

SSH 터널링에는 크게 로컬 터널링(Local Tunneling), 리모트 터널링(Remote Tunneling), 다이나믹 터널링(Dynamic Tunneling)이 있습니다.
암호화를 이용한 데이터 패킷 전송 등에 사용되는데, 여기서는 로컬 터널링과 리모트 터널링에 대해 설명해봅니다.

<br>

## 경유 서버 설정

터널링을 위해서는 경유 서버가 필요합니다. 경유 서버에서 다음 설정을 해줍니다.
`/etc/ssh/sshd_config` 파일에 다음 내용을 추가해줍니다.

<pre class="prettyprint">
GatewayPorts yes
AllowTcpForwarding yes
</pre>

그 이후

<pre class="prettyprint">
sudo service ssh restart
</pre>

명령어로 ssh 데몬 서비스를 다시 실행합니다.

<br>

## Local Tunneling

로컬에 있는 특정 포트를 다른 서버 주소로 연결하는 방식입니다.

다음과 같은 커맨드로 사용할 수 있습니다.

<pre class="prettyprint">
ssh -L sourcePort:forwardToHost:destPort connectToHost
ssh -L [로컬 PC 포트]:[타켓 서버 주소]:[타켓 서버 포트] [경유 서버 주소]

ex)
ssh -L 80:intra.example.com:80 gw.example.com
</pre>

위 명령어는 'gw.example.com' 서버를 경유 서버로 설정하고, 로컬 PC의 80 포트를 'ntra.example.com' 서버의 80 포트로 연결하는 
예제입니다.

조금 응용하면 다음과 같은 예제가 가능합니다.

아마존 서버(AWS)의 ssh 서버 포트가 기본적으로 22로 되어 있는데, 이를 8080 포트로 바꾸는 명령어는 다음과 같습니다.
이 경우는 AWS 서버가 타켓 서버가 됨과 동시에 로컬 PC 역할도 담당합니다.

먼저 아마존 서버에 접속한 다음

<pre class="prettyprint">
ssh -i snowdeer-key.pem -L 8080:localhost:22 localhost
</pre>

명령어를 수행하면 됩니다. 

이제 다른 PC에서 AWS 서버에 SSH 접속을 할 때 기존 22번 포트가 아닌 8080 포트를 사용해서 접속할 수 있습니다.

<pre class="prettyprint">
ssh -i snowdeer-key.pem -p 8080 ubuntu@xxx.xxx.xxx.xxx
</pre>

<br>

## Remote Tunneling

<pre class="prettyprint">
ssh -R sourcePort:forwardToHost:destPort connectToHost
ssh -R [오픈할 PC 포트]:[타켓 서버 주소]:[타켓 서버 포트] [경유 서버 주소]

ex)
ssh -R 9000:localhost:3000 user@example.com
</pre>

위 예제는 로컬 PC의 3000 포트를 경유 서버의 9000 번 포트에 바인딩하는 예제입니다.

이 명령어 실행한 다음 `example.com:9000` 주소로 패킷을 주고 받을 수 있습니다.