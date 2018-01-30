---
layout: post
title: Visual Studio Code에 Go 개발 환경 세팅
category: Go
tag: [Go]
---
# Visual Studio Code에 Go 개발 환경 세팅

[Visual Studio Code](https://code.visualstudio.com/)(이하 vscode)에서 Go 언어 개발 환경 세팅 방법을 포스팅 해봅니다.

<br>

## go 플러그인 설치

![Image](/assets/go/001.png)

먼저 vscode에서 위 이미지와 같이 'go' 플러그인을 설치합니다.

<br>

## 개발 디렉토리 설정

그리고 소스를 관리할 개발용 디렉토리를 설정합니다. 저같은 경우는 Windows에서는 `C:\Workspace\vscode_go`로 세팅했고, Linux에서는 `/home/snowdeer/Workspace/go` 아래에 설정했습니다.

그리고 `GOPATH` 환경 변수 설정을 해야 합니다. Windows에서는 

![Image](/assets/go/003.png)

환경 변수 편집 화면에서 `GOPATH` 항목을 등록해주면 되고, Linux에서는 터미널에서 `export` 명령어를 이용하면 됩니다.

~~~
export GOPATH="/home/snowdeer/Workspace/go"
~~~

그리고 해당 디렉토리에는 각각 `src`, `pkg`, `bin` 이름의 하위 디렉토리를 만들어줍니다.

<br>

## 추가 파일 설치

이제 vscode에서 Go 프로그래밍을 위한 실행 파일들을 다운로드하고 설치하는 작업을 합니다. 위에서 만든 디렉토리의 `src` 폴더 아래에 `main.go` 파일을 작성하고 vscode에서 열어봅니다.

그러면 vscode에서 아래 이미지와 같이 필요한 파일들을 설치할 것인지를 물어볼 것입니다.

![Image](/assets/go/002.png)

그냥 `Install All`을 선택해서 모든 파일들을 설치하면 됩니다. 모든 파일들을 설치하는데는 약 5분 정도의 시간이 걸릴 수 있습니다. 

~~~
Installing 9 tools at C:\Workspace\vscode_go\bin
  gopkgs
  go-outline
  go-symbols
  guru
  gorename
  godef
  goreturns
  golint
  dlv

Installing github.com/uudashr/gopkgs/cmd/gopkgs SUCCEEDED
Installing github.com/ramya-rao-a/go-outline SUCCEEDED
Installing github.com/acroca/go-symbols SUCCEEDED
Installing golang.org/x/tools/cmd/guru SUCCEEDED
Installing golang.org/x/tools/cmd/gorename SUCCEEDED
Installing github.com/rogpeppe/godef SUCCEEDED
Installing sourcegraph.com/sqs/goreturns SUCCEEDED
Installing github.com/golang/lint/golint SUCCEEDED
Installing github.com/derekparker/delve/cmd/dlv SUCCEEDED

All tools successfully installed. You're ready to Go :).
~~~


<br>

## 테스트 코드 및 실행

다음 코드로 실행 테스트를 해봅니다.

<pre class="prettyprint">
package main

import "fmt"

func main() {
	fmt.Println("Hello, snowdeer")
	fmt.Println("안녕. 스노우디어")
}
</pre>

코드 작성 후 <kbd>F5</kbd> 키를 눌러 실행을 해봅니다. 브레이크 포인트(Break Point)를 걸고 디버깅을 해볼 수도 있습니다.

![Image](/assets/go/005.png)