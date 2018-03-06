---
layout: post
title: CentOS 7에 Go 언어 설치하기
category: Go
tag: [Go]
---
# Install Golang on CentOS 7

CentOS 7 기준으로 Go 언어를 설치하는 방법입니다.

<br>

## Go 언어 다운로드

먼저 현재 릴리즈되어 있는 Go 언어 패키지 다운로드 주소를 [여기에서 확인](https://golang.org/dl/)합니다.

<pre class="prettyprint">
$ wget https://dl.google.com/go/go1.10.linux-amd64.tar.gz
</pre>

<br>

## Go 언어 설치

위에서 받은 tar 파일의 압축을 풀어줍니다.

<pre class="prettyprint">
$ sudo tar -C /usr/local -xvzf go1.10.linux-amd64.tar.gz
</pre>

그리고 작업하고자하는 디렉토리에 아래 디렉토리들을 생성해줍니다. (저같은 경우는 `/home/snowdeer/go/` 디렉토리 아래에서 작업합니다.)

<pre class="prettyprint">
$ mkdir bin

$ mkdir pkg

$ mkdir src
</pre>

<br>

## 환경 변수 설정

`/etc/profile.d/path.sh` 파일을 열고 아래 내용을 저장합니다.

<pre class="prettyprint">
export PATH=$PATH:/usr/local/go/bin
</pre>

또한 `/home/snowdeer/.bash_profile` 파일에도 다음 라인을 추가합니다.

<pre class="prettyprint">
export GOBIN="$HOME/go/bin"
export GOPATH="$HOME/go"
</pre>

또한 기존의 `PATH` 변수에 `GOBIN` 경로도 추가해줍니다.

그런다음 터미널에서 다음 명령어를 입력해서 환경 변수를 시스템에 적용합니다.

<pre class="prettyprint">
$ source /etc/profile

$ source ~/.bash_profile
</pre>

<br>

## 설치 확인

다음 예제 코드를 실행해서 Go 언어가 잘 동작하는지 확인합니다.

<pre class="prettyprint">
package main

import "fmt"

func main() {
    fmt.Printf("Hello, World!\n")
}
</pre>

그 이후 터미널에서 다음 명령어를 실행해봅니다. (아무거나 실행해봐도 됩니다.)

<pre class="prettyprint">
$ go run hello.go

$ go build hello.go

$ go install hello.go
</pre>