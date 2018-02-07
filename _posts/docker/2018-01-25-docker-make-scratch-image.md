---
layout: post
title: Scratch 이미지 생성하기
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Scratch 이미지 생성하기

Docker에서 아무 것도 들어있지 않은 비어있는 베이스 이미지를 `scratch 이미지`라고 합니다.

`dev/null` 장치를 이용해서 비어있는 `tar` 파일을 `docker import`하면 scratch 이미지가 생성됩니다.

~~~
$ tar cv --files-from /dev/null | sudo docker import - scratch

$ docker images

REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
scratch             latest              8a0ef0f0a9b8        6 seconds ago       0B
~~~

<br>

## Scratch 이미지에 프로그램 추가하기

scratch 이미지는 비어있는 이미지이기 때문에 컨테이너 생성이 되지 않습니다. 여기에 프로그램을 하나 추가해보도록 하겠습니다.

아래의 코드와 같이 간단한 C++ 프로그램을 작성합니다.

<pre class="prettyprint">
#include <stdio.h>

using namespace std;

int main() {
    printf("Hello. SnowDeer.\n");
    
    return 0;
}
</pre>

scratch 이미지에는 아무 라이브러리도 없기 때문에 빌드할 때는 반드시 정적(static) 바이너리로 빌드해야합니다.

~~~
$ g++ hello.cpp -static -o hello
~~~

<br>

## Dockerfile 생성

그리고 Dockerfile을 아래와 같이 작성합니다.

<pre class="prettyprint">
FROM scratch
ADD ./hello /hello
CMD ["/hello"]
</pre>

그런 다음 `docker build` 명령을 수행합니다.

~~~
$ docker build --tag hello:0.1 .

Sending build context to Docker daemon   1.83MB
Step 1/3 : FROM scratch
 --->
Step 2/3 : ADD ./hello /hello
 ---> Using cache
 ---> 0d32a1e91f48
Step 3/3 : CMD ["/hello"]
 ---> Running in 84a81208a7a9
Removing intermediate container 84a81208a7a9
 ---> 13ea232f71c9
Successfully built 13ea232f71c9
Successfully tagged hello:0.1
~~~

이미지 생성 결과 확인은 다음과 같습니다.

~~~
$ docker images

REPOSITORY          TAG                 IMAGE ID            CREATED             SIZE
hello               0.1                 13ea232f71c9        46 seconds ago      913kB
scratch             latest              8a0ef0f0a9b8        7 minutes ago       0B
~~~

컨테이너로 실행을 해봅니다.

~~~
$ docker run --rm hello:0.1

Hello. SnowDeer.
~~~