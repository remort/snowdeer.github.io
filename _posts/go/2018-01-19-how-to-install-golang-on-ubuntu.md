---
layout: post
title: Ubuntu에 Go 언어 설치하기
category: Go
tag: [Go]
---
# Ubuntu에 Go 언어 설치하는 방법

Go 언어 공식 다운로드는 [https://golang.org/dl/](https://golang.org/dl/)에서 할 수 있습니다.

또는 Ubuntu 기준으로 다음 명령어로 Go 언어를 설치할 수도 있습니다.

<br>

## apt-get을 이용한 설치

~~~
$ sudo apt-get install golang-go
~~~

만약 너무 오래된 버전의 Go 언어가 설치될 경우 다음 명령어로 좀 더 최신 버전의 Go 언어를 설치할 수도 있습니다. (Ubuntu 16.04 LTS 버전의 경우는 아래쪽 포스팅 내용을 따르세요.)

~~~
$ sudo add-apt-repository ppa:gophers/archive

$ sudo apt-get update

$ sudo apt-get install golang-1.9-go
~~~

참고로 `golang-1.9-go` 패키지의 바이너리는 `/usr/lib/go-1.9/bin` 위치에 설치됩니다.


<br>

## snap을 이용한 설치

snap을 이용할 경우 다음 명령어를 이용해서 최신 버전의 Go 언어를 설치할 수 있습니다.

~~~
$ snap install --classic go
~~~

<br>

## Ubuntu 16.04 에서의 설치

Ubuntun 16.04에서는 `golang-1.9-go` 패키지를 설치 못하는 경우가 발생할 수 있습니다. 이런 경우는 다음 명령어를 이용해서 설치합니다.

~~~
$ sudo add-apt-repository ppa:longsleep/golang-backports

$ sudo apt-get update

$ sudo apt-get install golang-go
~~~