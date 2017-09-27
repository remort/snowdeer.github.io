---
layout: post
title: gdb 사용법
category: C++
tag: [C++]
---
# gdb를 이용한 디버깅

`gdb`는 프로그램 실행 도중 지정된 브레이크 포인트(Breakpoint)에서 멈출 수 있으며, 해당 지점에서 각 변수들이나 포인터 들의 값을 확인할 수 있는 디버깅 툴입니다. 

다음과 같이 사용할 수 있습니다.

~~~
$ gdb [실행파일명]
~~~

`gdb`를 실행하면 명령어를 입력받을 수 있는 콘솔 프롬프트(prompt)가 실행됩니다. `help`를 입력해서 도움말을 확인할 수 있고 종료하고 싶은 경우는 `quit`, 또는 <kbd>Ctrl</kbd> + <kbd>D</kbd>를 입력하면 됩니다.

<br>

## Breakpoint 설정

gdb 콘솔에서 `break` 명령어(단축 명령어 : `b`)를 입력하면 됩니다. 다음과 같이 설정할 수 있습니다.

* break [파일명:]함수명
* break [파일명:]라인 번호

설정된 Breakpoint는 `info break` 명령어를 입력해서 조회할 수 있습니다. 또한 `clear` 또는 `delete` 명령어를 이용하여 Breakpoint를 해제하거나 삭제할 수 있습니다.

명령어 | 단축 명령어 | 설명
---|---|---
break | b | Breakpoint 설정
clear | |설정된 Breakpoint를 해제
delete | d | 설정된 Breakpoint를 삭제
info break | info b | Breakpoint 정보 조회

<br>

## 프로그램 실행

Breakpoint를 설정한 다음 프로그램 실행은 `run(r)` 명령어로 할 수 있습니다. 또한 다음 명령어들을 통해서 각각의 명령을 내릴 수 있습니다.

명령어 | 단축 명령어 | 설명
---|---|---
continue | c | 다음 Breakpoint 까지 코드를 수행
step | s | 한 줄씩 코드를 수행하며, 함수 내부까지 들어감
next | n | 한 줄씩 코드를 수행하며, 함수는 건너 뜀
kill | | 프로그램의 수행을 종료

`s`나 `n`을 입력한 다음부터는 <kbd>Enter</kbd> 키만 누르면 이전에 실행했던 명령이 다시 실행됩니다. 현재 시점에서 소스 코드를 확인하기 위해서는 `list(l)` 명령어를 이용하면 됩니다.

<br>

## 변수 값 확인

현재 정지된 위치에서 각 변수들의 값을 확인하기 위해서는 `print [변수명]` 명령어를 이용하면 됩니다. `print`의 단축 명령어는 `p` 입니다. 라인을 이동하면서 계속 변수 값을 확인할 때는 `display [변수명]` 명령어를 이용하면 됩니다. 또한 `delete display [변수명]` 명령어를 이용해 해당 변수의 display 명령을 해제할 수 있습니다.

<br>

## 변수 값 변경

`print str="abc"`와 같은 형태의 명령을 통해 특정 변수의 값을 변경할 수 있습니다.