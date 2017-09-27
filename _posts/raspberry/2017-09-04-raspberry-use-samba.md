---
layout: post
title: samba를 이용한 파일 공유
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# samba를 이용하여 라즈베리파이를 NAS처럼 사용하기

`samba`를 이용하면 라즈베리파이의 파일들을 보다 쉽게 공유할 수 있습니다. 특히 라즈베리파이에 USB 메모리 등을 연결했을 때, 그 USB 메모리를 마치 NAS처럼 사용할 수도 있습니다.

<br>

## samba 설치

다음 명령어를 이용해서 `samba`를 설치합니다.

~~~
$ sudo apt-get install samba
$ sudo apt-get install samba-common-bin
~~~

<br>

## USB 메모리를 연결

라즈베리파이에 USB 메모리를 연결하면 자동으로 `/media`에 마운트 됩니다.

<br>

## samba 사용자 추가

`samba`에 사용자 `pi`를 추가하는 명령어입니다.

~~~
$ sudo smbpasswd -a pi
New SMB password:
Retype new SMB password:
Added user pi.
~~~

<br>

## Config 파일 수정

그 후 `/etc/samba/smb.conf` 파일을 `nano`로 열어서 편집을 합니다.

~~~
workgroup = WORKGROUP
~~~

이 부분은 Windows에서 연결하는 경우에만 변경해주면 됩니다.

사용자 인증 세션 부분은 다음과 같습니다.

~~~
# security = user
~~~

이 부분에서 앞의 `#`을 제거하여 보안 설정을 활성화합니다.

그리고 파일 맨 뒤에 다음 내용을 추가합니다.

~~~
[USB]
path = /media/workspace
comment = USB File Sharing
valid users = pi
writeable = yes
browseable = yes
create mask = 0777
public = yes
~~~

파일을 저장한 다음 다음 명령어로 samba 서버를 재시작합니다.

~~~
sudo /etc/init.d/samba restart
~~~