---
layout: post
title: [Fedora] gcc: error: /usr/lib/rpm/redhat/redhat-hardened-cc1: No such file or directory 오류 발생시
category: Linux
tag: [리눅스 명령어, Fedora]
---
# redhat-rpm-config 설치

Fedora에서 'gcc: error: /usr/lib/rpm/redhat/redhat-hardened-cc1: No such file or directory' 오류가 발생할 경우 다음 명령어를 이용해서 `redhat-rpm-config`를 설치해주면 해결됩니다.

~~~
dnf install redhat-rpm-config
~~~