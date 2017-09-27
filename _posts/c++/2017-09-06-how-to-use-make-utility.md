---
layout: post
title: make를 이용한 빌드 방법
category: C++
tag: [C++]
---
# make 사용법

## gcc를 이용한 빌드 방법

일반적으로 `gcc`를 이용한 빌드는 다음과 같은 형태의 명령을 이용합니다.

~~~
$ gcc -o helloworld helloworld.c
~~~

이 때 `-o` 옵션 뒤에는 실행 파일의 이름을 지정합니다.

하지만 실제로 프로젝트를 진행하면 수 많은 소스 코드를 사용하는데, 일일이 `gcc`를 이용해서 빌드하는 것은 번거롭기 때문에 `make` 유틸리티를 활용하면 보다 효율적으로 빌드를 할 수 있습니다.

`make`를 사용하기 위해서는 `makefile`이 필요합니다. `makefile`이 있는 경우에는 콘솔창에서 `make`만 입력해도 자동으로 빌드가 되며, 별도로 `--makefile` 또는 `-f` 옵션을 이용하여 `makefile`의 이름을 지정할 수도 있습니다.

<br>

## makefile의 구성

`makefile`은 크게 목표(Target), 의존성(Dependency), 명령(Command)의 3 부분으로 이루어집니다.

~~~
target:         dependencyList
                commandList
~~~

요소 | 설명
---| ---
targetLit | 명령이 수행되어 생성될 결과 파일 지정
dependencyList | target(목표)을 수행하기 위해 필요한 의존 관계 설정
commandList | target을 수행하기 위해 여기에 정의된 명령이 실행됨

Command(명령)의 경우 다양한 유틸리티를 사용할 수 있습니다. 예를 들어 `gcc` 외에도 `clear`, `cp` 등의 대부분의 명령어를 사용할 수 있습니다. 명령의 앞 부분은 반드시 <kbd>Tab</kbd>을 이용해서 공백을 만들어주어야 합니다.

<br>

## make 예제

다음과 같은 파일들이 존재한다고 가정해봅시다.

* main.c
* input.c
* output.c

위의 파일들은 `gcc`를 이용했을 때 다음과 같이 빌드할 수 있습니다.

~~~
$ gcc -o snowdeerApp main.c input.c output.c
~~~

이 경우 `make`를 위한 `makefile`로 만들어보면 다음과 같습니다.

~~~
snowdeerApp :
    gcc -o snowdeerApp main.c input.c output.c
~~~

기본적으로 위와 같은 `makefile`을 이용하면 되지만, 향후 확장을 위해 다음과 같이 좀 더 유연하게 만들어 줄 수 있습니다.

~~~
snowdeerApp : main.o input.o output.o
    gcc -o snowdeerApp main.o input.o output.o

main.o : main.c
    gcc -c main.c

input.o : input.c
    gcc -c input.c

output.o : output.c
    gcc -c output.c
~~~

<br>

## makefile의 매크로 사용

위에서 작성한 `makefile`에 '$(매크로)' 같은 형식의 매크로 기능을 사용할 수 있습니다.

~~~
OBJECTS = main.o input.o output.o

snowdeerApp : $(OBJECTS)
    gcc -o snowdeerApp $(OBJECTS)

main.o : main.c
    gcc -c main.c

input.o : input.c
    gcc -c input.c

output.o : output.c
    gcc -c output.c
~~~

`makefile`을 좀 더 수정하면

~~~
OBJECTS = main.o input.o output.o
SRC = main.c input.c output.c

CFLAGS = -g
TARGET = snowdeerApp

$(TARGET) : $(OBJECTS)
    $(CC) -o $(TARGET) $(OBJECTS)

clear :
    rm -f $(OBJECTS) $(TARGET) core

main.o : main.c
input.o : input.c
output.o : output.c
~~~

와 같은 형태로 사용할 수 있습니다.

<br>

## make에 이미 정의되어 있는 키워드들

위에서 `$(CC)`나 `CFLAG` 등의 키워드들은 `make`에 이미 정의되어 있는 매크로입니다.

매크로 | 설명
---|---
ARFLAGS | ar 아카이브 관리 프로그램의 Flag
ASFLAGS | as 어셈블러의 Flag
CFLAGS | C 컴파일러의 Flag
CXXFLAGS | C++ 컴파일러의 Flag
CPPFLAGS | C 언어 전처리기(Preprocessor)의 Flag
LDFLAGS | ld linker의 Flag
COFLASG | co 유틸리티의 Flag
FFLAGS | 포트란 컴파일러의 Flag
PFLAGS | 파스칼 컴파일러의 Flag
LFLAGS | lex의 Flag
YFLAGS | yacc의 Flag

명령어 매크로 | 명령어 | 설명
---|---|---
AR | ar | ar 아카이브 관리 프로그램
AS | as | as 어셈블러
CC | cc | C 언어 컴파일러
CXX | g++ | C++ 컴파일러
CO | co | co 유틸리티
CPP | $(CC) -E | C 언어 전처리기(Preprocessor)
FC | f77 | 포트란 컴파일러
PC | pc | 파스칼 컴파일러
LEX | lex | lex 프로세서
YACC | yacc | yacc 프로세서
TEX | tex | tex 프로세서
RM | rm -f | 파일 삭제

<br>

## 자동 변수

먼저 다음과 같은 예제를 살펴보겠습니다.

~~~
.SUFFIXES : .c .o

OBJECTS = main.o input.o output.o
SRC = main.c input.c output.c

CC = gcc
CFLAGS = -g
TARGET = snowdeerApp

$(TARGET) : $(OBJECTS)
    $(CC) -o $@ $(OBJECTS)

.c.o :
    $(CC) $(CFLAGS) -c $< -o $@

clear :
    $(RM) -f $(OBJECTS) $(TARGET) core

main.o : main.c
input.o : input.c
output.o : output.c
~~~

위에서 `$(TARGET)` 명령에서 빌드 옵션에 보면 `$@`가 있습니다. 이 항목은 현재 target으로 하는 대상의 이름을 지칭하는 옵션입니다. 그 외의 주요 자동 변수들은 다음과 같습니다.

변수 | 설명
---|---
$* | 확장자가 없는 현재 목표 파일의 이름을 지칭
$@ | 현재 목표 파일의 이름을 지칭
$< | 현재 목표 파일보다 더 최근에 갱신된 파일명으로, 첫 번째 종속물의 이름
$? | '$<'와 동일

목표로 선언한 `.c.o`는 `.o`에 대응하는 `.c`를 발견하면 해당 command가 수행됩니다. `$<`는 확장자를 제외한 파일의 이름과 target(`.c.o`)의 앞 부분인 `.c`를 붙여서 사용되며, 뒤의 `$@`는 확장자를 제외한 파일의 이름과 target 뒤의 `.o`를 붙여서 사용됩니다.

<br>

## 매크로 치환

매크로 치환은 다음과 같은 명령어로 할 수 있습니다.

~~~
$(매크로 이름:이전 내용=새로운 내용)
~~~

예를 들어, 다음과 같은 형태로 사용할 수 있습니다.

~~~
# 생략
OBJECTS = main.o input.o output.o

# 생략
SRC = $(OBJECTS:.o=.c)

# 생략
~~~

<br>

## 다중 타켓

실행 파일을 여러 개 생성할 때 사용하는 방법입니다. `makefile` 안에 `all`이라는 target을 만들고 그 뒤에 필요한 target들을 나열하면 됩니다. 단, `all`의 앞에 다른 target을 넣으면 안됩니다.

~~~
# 생략
# TARGET = snowdeerApp

all : snowdeerApp helloWorld

snowdeerApp : $(OBJECTS)
    $(CC) -o $@ $(OBJECTS)

helloWorld : helloworld.c
    $(CC) -o $@ $<

# 생략
~~~