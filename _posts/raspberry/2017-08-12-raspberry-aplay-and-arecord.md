---
layout: post
title: arecord 및 aplay 사용방법
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# alsa-utils 설치

사운드를 녹음하는 `arecord`와 사운드를 재생하는 `aplay` 프로그램은 `alsa-utils`에 포함되어 있습니다.

~~~
sudo apt-get install alsa-utils
~~~

명령어로 두 프로그램을 설치할 수 있습니다.

<br>

## arecord 사용법

`arecord`는 다음과 같이 사용할 수 있습니다.

~~~
arecord -t raw -c 1 -D plughw:1,0 -f S16_LE -d 5 -r 16000 test.pcm
~~~

`-t` 옵션은 타입으로 다음과 같은 타입을 가집니다.

* voc
* wav
* raw
* au

`-c` 옵션은 채널 개수를 의미합니다.

`-D` 옵션은 디바이스 이름을 의미하며, 위 예제에서는 `plughw:1,0`이라는 디바이스 이름을 사용했지만, 라즈베리파이에서는 보통 `default`라는 이름으로 세팅되어 있습니다.

`-f` 옵션은 포맷(format)을 의미하며 보통 `S16_LE` 값을 사용하면 됩니다.

`-d`는 'duration'을 의미합니다. 위와 같이 사용하면 5초 동안 녹음합니다.

`-r` 옵션은 'sample-rate' 값입니다.

<br>

## aplay 사용법

위에서 녹음한 파일을 재생할 때는 다음과 같은 옵션으로 재생하면 됩니다.

~~~
aplay -t raw -c 1 -f S16_LE -r 16000 test.pcm
~~~

옵션이 의미하는 바는 `arecord`와 같습니다.

만약 `pcm` 파일이 아닌 일반 `wav` 파일이면 그냥 

~~~
aplay filename
~~~

으로 실행해도 됩니다. `wav` 파일 안에 헤더 정보가 들어있기 때문입니다.