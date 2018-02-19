---
layout: post
title: 모든 컨테이너를 Stop & Remove 하는 방법
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Stop and Remove all Docker Container

모든 컨테이너를 Stop 하고 Remove 하는 방법입니다.

~~~
docker stop $(docker ps -a -q)

docker rm $(docker ps -a -q)
~~~