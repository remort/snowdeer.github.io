---
layout: post
title: Data Volume 사용하는 방법
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker의 파일 시스템

Docker에서 컨테이너 내부의 파일들은 'Union File System'에 의해 컨테이너 내부에서 관리됩니다. 하지만 'Data Volume'을 사용하게 되면 호스트 PC의 파일 시스템에 파일을 저장할 수가 있습니다. 따라서 이 경우에는 `docker commit` 명령어를 이용하더라도 해당 파일은 이미지에 포함되지 않습니다.

<br>

## Data Volume 설정 방법

볼륨을 연결하는 옵션은 `-v <호스트 PC 디렉토리>:<컨테이너 디렉토리>` 입니다. 아래 예제는 로컬 호스트 PC의 `/home/snowdeer/Docker/data-volume/` 디렉토리를 `snow-volume`라는 이름의 컨테이너 내부의 `/data` 디렉토리로 연결할 수 있습니다.

~~~
$ docker run -i -t --name snow-volume -v /home/snowdeer/Docker/data-volume/:/data ubuntu /bin/bash
~~~

위 명령어를 실행하고 나서 bash 명령어로 

~~~
$ cd /data

$ touch hello-snowdeer.txt
~~~

를 입력하고 나서 호스트 PC의 연결된 디렉토리를 살펴보면 컨테이너 내부에서 생성한 파일이 보이는 것을 확인할 수 있습니다.

이렇게 데이터 볼륨을 이용하면 여러 컨테이너들끼리 같은 파일 데이터를 공유할 수 있습니다. 또한 디렉토리 연결뿐만 아니라 다음과 같이 파일을 연결할 수도 있습니다.

~~~
$ docker run -i -t --name snow-volume -v /home/snowdeer/Docker/data-volume/hello.txt/:/data/hello.txt ubuntu /bin/bash
~~~

<br>

## --volumes-from 옵션을 이용한 데이터 볼륨 연결

다음 명령어를 입력하면 위에서 생성한 'snow-volume'이라는 이미지의 데이터 볼륨을 새로 생성하는 컨테이너에 연결할 수 있습니다.

~~~
$ docker run -i -t --volumes-from snow-volume --name snow-volume2 ubuntu /bin/bash
~~~