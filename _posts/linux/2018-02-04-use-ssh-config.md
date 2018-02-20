---
layout: post
title: ssh Config 사용법
category: Linux
tag: [Linux, ssh]
---
# ssh config

`ssh` 접속을 하기 위해서는 보통 `ssh [사용자ID]@[Target IPAddress]`와 같이 사용하거나 그 외 포트, 키(Key) 등을 옵션으로 해서 같이 사용해야 하는 경우가 많아서 번거롭습니다.

~~~
ex) 
ssh snowdeer@10.13.152.45

ssh snowdeer@10.13.152.45 -i ~/.ssh/snowdeer_aws.pem
~~~

이럴 때 'ssh config'를 사용하면 보다 간편하게 `ssh`를 사용할 수 있습니다.

<br>

# ~/.ssh/config 설정 파일

'ssh config' 파일의 위치는 `~/.ssh/config` 입니다. 폴더나 파일이 존재하지 않는다면 직접 만들면 됩니다. 해당 파일을 생성하고 권한은 다음과 같이 부여합니다.

<pre class="prettyprint">
chmod 440 ~/.ssh/config
</pre>

그리고 `config` 파일 내용은 다음과 같이 작성합니다.

<pre class="prettyprint">
Host master
    HostName 10.13.152.45
    User ubuntu
    IdentityFile ~/.ssh/snowdeer-aws-seoul.pem

Host node1
    HostName 10.13.152.113
    User ubuntu
    IdentityFile ~/.ssh/snowdeer-aws-seoul.pem

Host node2
    HostName 10.13.11.53
    User ubuntu
    IdentityFile ~/.ssh/snowdeer-aws-seoul.pem
</pre>

<br>

# config 파일 사용법

사용법은 단순합니다. 단순히 `ssh <host name>`으로 명령을 내리면 됩니다.

<pre class="prettyprint">
ex) ssh node1
</pre>