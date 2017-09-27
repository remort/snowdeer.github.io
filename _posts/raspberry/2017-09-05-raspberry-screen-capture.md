---
layout: post
title: scrot를 이용한 화면 캡쳐
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# 화면 캡쳐 프로그램 scrot

## scrot 설치

다음 명령어를 이용하여 `scrot` 프로그램을 설치합니다.

~~~
$ sudo apt-get install scrot
~~~

<br>

## 화면 캡쳐 방법

가장 간단하게 하는 방법으로는 단순히 `scrot` 명령어를 실행하는 것입니다. 화면이 캡쳐되면 현재 디렉토리 안에 `png` 형식으로 파일이 저장됩니다.

`-d` 옵션을 이용하면 화면 캡쳐에 딜레이를 설정 할 수 있습니다.

~~~
$ scrot -d 5
~~~

화면의 일부만 캡쳐하고 싶을 때는 `-s` 옵션을 줘서 마우스도 드래그하여 영역을 설정할 수도 있습니다.