---
layout: post
title: 사용자 계정 관리하기
category: Linux
tag: [Linux 명령어, Account]
---

## 계정 생성
리눅스에서 사용자 계정은 다음 명령어를 이용해서 생성할 수 있습니다.
기본적으로 -m 옵션을 붙여서 사용자의 홈 디렉토리(Home Directory)를 생성할 수 
있습니다.

~~~
sudo useradd -m [사용자 ID]

ex) sudo useradd -m snowdeer
~~~

계정을 생성한 다음에는 `ls /home` 명령어를 이용해서 확인할 수 있습니다.


<br>

## 패스워드 설정
패스워드는 `passwd` 명령어를 이용해서 설정할 수 있습니다.

~~~
sudo passwd snowdeer

ex)
pi@snowdeer_raspberry:~ $ sudo passwd snowdeer
Enter new UNIX password: ********
Retype new UNIX password: ********
passwd: password updated successfully
~~~

<br>
## 패스워드 변경

현재 로그인해 있는 계정의 패스워드를 변경할 경우는 다음과 같이 `passwd` 명령어만
입력하면 됩니다.

~~~
sudo passwd
~~~


<br>

## 계정 삭제

계정 삭제는 `userdel` 명령어로 가능합니다.
~~~
sudo userdel snowdeer
~~~

계정을 삭제해도 `/home` 밑에 홈 디렉토리는 남아 있기 때문에 수작업으로 
삭제를 해주시면 됩니다.