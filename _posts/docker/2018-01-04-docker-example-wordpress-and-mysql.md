---
layout: post
title: Wordpress 이미지와 MySQL 이미지 연결(Link) 예제
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker 컨테이너간 연결

Docker로 이미지를 생성할 때, 하나의 이미지에 웹 서버, 데이터베이스 등 필요한 프로그램을 모두 설치하는 것도 가능하지만, 보통은 프로그램별로 이미지를 따로 생성하는 경우가 많습니다. 이렇게 이미지를 따로 생성하면 나중에 웹 서버를 교체하거나 데이터베이스를 교체하는 일이 있을 때 각 서버간 충격을 완화시키고 훨씬 더 유연하게 대처할 수 있기 때문입니다.

Docker 컨테이너끼리 연결할 때는 `docker run` 명령어에 `--link` 옵션을 사용합니다.

<br>

## Wordpress 및 MySQL

Docker Hub에는 Wordpress 이미지가 이미 존재합니다. 따라서 해당 이미지를 내려받아서 설치하면 간단하게 Docker 기반 Wordpress 컨테이너를 생성할 수 있습니다. 하지만 Wordpress는 MySQL 데이터베이스를 필요로 합니다. 따라서 MySQL 이미지도 내려받아야 하며, 두 이미지로 생성한 컨테이너간 링크(Link) 설정을 해주어야 합니다.

컨테이너간 링크는 'Wordpress' 컨테이너에서 'MySQL' 컨테이너로 연결을 해줘야 하기 때문에, 여기서는 편의상 MySQL 컨테이너를 먼저 실행하고 그 이후 Wordpress 컨테이너를 실행하도록 하겠습니다.

<br>

## MySQL 이미지 다운로드 및 컨테이너 실행

~~~
$ docker run --name snow-mysql -e MYSQL_ROOT_PASSWORD=snowdeer -d mysql 
~~~

MySQL 이미지는 몇 가지 파라메터(특히 ROOT 패스워드)를 설정해줘야 정상적으로 동작합니다. `-e` 옵션을 이용해서 필수 변수의 값을 세팅해줬습니다. 해당 옵션은 Docker HUB 사이트의 [MySQL 이미지 페이지]((https://hub.docker.com/_/mysql/)에서 확인할 수 있습니다. 만약 파라메터없이 `run` 명령어로 수행했을 때도, `docker logs <container name>` 명령어를 통해서 해당 컨테이너가 정상적으로 돌고 있는지, 또는 오류가 발생했는지 확인할 수 있습니다.

`-d` 옵션은 'detach'라는 의미로 백그라운드에서 컨테이너가 돌 수 있도록하는 옵션입니다.

<br>

## Wordpress 컨테이너 실행 및 링크 설정

~~~
$ docker run --name snow-wp --link snow-mysql:mysql -p 80 -d wordpress
~~~

위 명령어는 `snow-wp`라는 이름의 컨테이너를 생성하며, `mysql` 이미지로 생성된 `snow-mysql`이라는 이름의 컨테이너에 링크(link) 설정을 합니다. 이 때 링크되는 포트는 80번 포트가 되며 `-d` 옵션을 통해 백그라운드로 수행됩니다. 마지막의 `wordpress`는 `snow-wp` 컨테이너를 생성할 이미지 이름입니다.

<br>

## 컨테이너 상태 조회

~~~
$ docker ps

CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS                   NAMES
4c74627deef3        wordpress           "docker-entrypoint.s…"   13 minutes ago      Up 13 minutes       0.0.0.0:32768->80/tcp   snow-wp
ef5202166d02        mysql               "docker-entrypoint.s…"   15 minutes ago      Up 15 minutes       3306/tcp                snow-mysql
~~~

위와 같이 컨테이너 2개가 동작하고 있는 것을 확인할 수 있으며, `snow-wp` 컨테이너의 포트 정보를 보면 `0.0.0.0:32768`가 80번 포트로 연결되었음을 확인할 수 있습니다.

<br>

## 웹브라우저에서 Wordpress 확인

위에서 확인한 `0.0.0.0:32768` 정보로 웹 브라우저에서 접속하면 다음과 같이 Wordpress 설정 화면을 볼 수 있습니다.

![Image](/assets/docker/004.png)