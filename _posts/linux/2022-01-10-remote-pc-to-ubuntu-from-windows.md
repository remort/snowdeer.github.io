---
layout: post
title: Windows에서 Ubuntu로 원격 접속하기

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# Remote PC (Windows -> Ubuntu)

Windows에서 Ubuntu PC에 원격 접속할 수 있도록 설정하는 방법입니다.
환경은 `Windows 10` 및 `Ubuntu 20.04` 입니다.

## xrdp 설치

<pre class="prettyprint">
sudo apt update
sudo apt install xrdp
sudo systemctl enable --now xrdp
sudo ufw allow from any to any port 3389 proto tcp
</pre>

그 이후 Windows의 `원격데스크탑 접속`을 이용해서 리눅스에 접속하면 되고, 
다이얼로그 창이 뜨면, `Xorg` Session, 계정, 패스워드를 입력하면 됩니다.

## 만약 접속시 Black Screen 만 나오는 경우

`/etc/xrdp/startwm.sh` 파일을 수정합니다.

<pre class="prettyprint">
sudo gedit /etc/xrdp/startwm.sh
</pre>

그리고 여기에 아래 내용을 입력합니다.

<pre class="prettyprint">
unset DBUS_SESSION_BUS_ADDRESS
unset XDG_RUNTIME_DIR
. $HOME/.profile
</pre>

## startwm.sh 전체 파일 내용

<pre class="prettyprint">
#!/bin/sh
# xrdp X session start script (c) 2015, 2017 mirabilos
# published under The MirOS Licence

if test -r /etc/profile; then
	. /etc/profile
fi

if test -r /etc/default/locale; then
	. /etc/default/locale
	test -z "${LANG+x}" || export LANG
	test -z "${LANGUAGE+x}" || export LANGUAGE
	test -z "${LC_ADDRESS+x}" || export LC_ADDRESS
	test -z "${LC_ALL+x}" || export LC_ALL
	test -z "${LC_COLLATE+x}" || export LC_COLLATE
	test -z "${LC_CTYPE+x}" || export LC_CTYPE
	test -z "${LC_IDENTIFICATION+x}" || export LC_IDENTIFICATION
	test -z "${LC_MEASUREMENT+x}" || export LC_MEASUREMENT
	test -z "${LC_MESSAGES+x}" || export LC_MESSAGES
	test -z "${LC_MONETARY+x}" || export LC_MONETARY
	test -z "${LC_NAME+x}" || export LC_NAME
	test -z "${LC_NUMERIC+x}" || export LC_NUMERIC
	test -z "${LC_PAPER+x}" || export LC_PAPER
	test -z "${LC_TELEPHONE+x}" || export LC_TELEPHONE
	test -z "${LC_TIME+x}" || export LC_TIME
	test -z "${LOCPATH+x}" || export LOCPATH
fi

unset DBUS_SESSION_BUS_ADDRESS
unset XDG_RUNTIME_DIR
. $HOME/.profile

if test -r /etc/profile; then
	. /etc/profile
fi

test -x /etc/X11/Xsession && exec /etc/X11/Xsession
exec /bin/sh /etc/X11/Xsession
</pre>
