---
layout: post
title: Docker 유저 권한 추가(sudo 없이 Docker 명령어 실행하는 방법)
category: Docker
permalink: /blog/:year/:month/:day/:title/

tag: [Docker]
---
# 유저 권한 추가

Docker 명령어를 실행하기 위해서는 매번 `sudo`를 입력해야 합니다. `sudo` 명령어 없이 Docker를 사용하기 위해서는 Docker가 사용하는 그룹에 사용자 ID를 추가하면 됩니다.

~~~
sudo gpasswd -a <사용자ID> docker

$ sudo gpasswd -a snowdeer docker
~~~

그리고 나서 로그아웃 후 로그인하면 됩니다.