---
layout: post
title: Nginx 이미지 생성하기
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# nginx

[nginx](https://nginx.org/en/)는 가볍고 실용적인 웹서버입니다. 웹서버, 리버스 프록시, 메일 프록시 등의 기능을 갖고 있습니다.

<br>

# ngix 이미지 생성하기

nginx 이미지 생성용 Dockerfile은 [여기](https://github.com/dockerfile/nginx/blob/master/Dockerfile)에서 확인할 수 있습니다.

<pre class="prettyprint">
FROM ubuntu:latest

# Install Nginx.
RUN apt-get update
RUN apt-get install -y nginx
RUN echo "\ndaemon off;" >> /etc/nginx/nginx.conf
RUN chown -R www-data:www-data /var/lib/nginx

# Define mountable directories.
VOLUME ["/etc/nginx/sites-enabled", "/etc/nginx/certs", "/etc/nginx/conf.d", "/var/log/nginx", "/var/www/html"]

# Define working directory.
WORKDIR /etc/nginx

# Define default command.
CMD ["nginx"]

# Expose ports.
EXPOSE 80
EXPOSE 443
</pre>

위와 같은 Dockerfile을 생성하고 해당 폴더에서 아래 명령어를 이용해서 nginx 이미지를 빌드할 수 있습니다.

~~~
$ docker build --tag snow-nginx:0.1 .

Sending build context to Docker daemon  2.048kB
Step 1/10 : FROM ubuntu:latest
latest: Pulling from library/ubuntu
1be7f2b886e8: Pull complete
6fbc4a21b806: Pull complete
c71a6f8e1378: Pull complete
4be3072e5a37: Pull complete
06c6d2f59700: Pull complete
Digest: sha256:e27e9d7f7f28d67aa9e2d7540bdc2b33254b452ee8e60f388875e5b7d9b2b696
Status: Downloaded newer image for ubuntu:latest
 ---> 0458a4468cbc
Step 2/10 : RUN apt-get update
 ---> Running in 5ff5327e54cb

...

Step 9/10 : EXPOSE 80
 ---> Running in da53118109a4
Removing intermediate container da53118109a4
 ---> 395dc7dc3371
Step 10/10 : EXPOSE 443
 ---> Running in 3f8c3f9a8895
Removing intermediate container 3f8c3f9a8895
 ---> a935dd536d8e
Successfully built a935dd536d8e
Successfully tagged snow-nginx:0.1
~~~

<br>

빌드된 이미지 확인은 `docker images`로 확인할 수 있습니다.

~~~
$ docker images

REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
snow-nginx          0.1                 a935dd536d8e        58 seconds ago      208MB
~~~