---
layout: post
title: 명령어들의 설치 위치 확인
category: Linux
tag: [Linux]
---
# 명령어들의 설치 위치 확인

명령어들의 설치 위치 확인은 다음 명령어를 이용해서 확인할 수 있습니다.

## type

<pre class="prettyprint">
$ type go
go is hashed (/snap/bin/go)

$ type ifconfig
ifconfig is /sbin/ifconfig
</pre>

<br>

## which

<pre class="prettyprint">
$ which go
/snap/bin/go
</pre>

<br>

## whereis

<pre class="prettyprint">
$ whereis go
go: /snap/bin/go.gofmt /snap/bin/go
</pre>
