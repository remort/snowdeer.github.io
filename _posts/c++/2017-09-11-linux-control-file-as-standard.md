---
layout: post
title: Linux에서 표준 파일 입출력
category: C++
tag: [C++, Linux]
---
# 표준 입출력 라이브러리 및 스트림

초창기 유닉스(Unix) 시스템에서는 모니터 대신 프린터, 저장 장치로는 자기 테이프 등이 이용되었는데 입출력 속도가 너무 느렸습니다. 그러다보니 조금이라도 속도를 향상시키기 위해서 표준 입출력 라이브러리(Standard Input/Output Library)가 등장하게 되었습니다. 

표준 입출력 라이브러리는 입출력을 위해 데이터를 바로 사용하지 않고, 버퍼(Buffer)를 이용해서 데이터를 한꺼번에 처리하도록 했습니다. 그래서 파일의 입출력에 스트림(Stream)을 사용할 수 있도록 했습니다.

<br>

## FILE 구조체

FILE 구조체라는 파일 포인터를 이용해서 파일을 제어할 수 있습니다. FILE 구조체는 `<stdio.h>` 헤더 파일에 정의되어 있습니다.

<pre class="prettyprint">
typedef struct {
  int _cnt;
  unsigned char *_ptr;
  unsigned char *_base;
  unsigned char _flag;
  unsigned char _file;
} FILE;
</pre>

FILE 구조체는 스트림을 다루기 위한 파일 디스크립터, 버퍼 공간에 대한 포인터, 버퍼 크기, 버퍼에 남아 있는 문자 수, 에러 Flag 등의 정보를 갖고 있습니다.

<br>

## fopen()

파일을 읽거나 쓰기 위해 파일을 여는 기능을 합니다.

<pre class="prettyprint">
FILE *fopen(const char *path, const char *mode);
</pre>

두 번째 인자인 `mode`에는 다음과 같은 값들이 들어갈 수 있습니다.

인자 | 읽기 | 쓰기 | 파일 생성
---|---|---|---
r | O | X | X
r+ | O | O | X
w | X | O | O
w+ | O | O | O
a | X | O | O
a+ | O | O | O

<br>

## fclose()

사용한 파일을 닫습니다.

<pre class="prettyprint">
int fclose(FILE *fp);
</pre>

<br>

## fread() / fwrite

파일을 읽거나 쓰는 기능을 수행합니다.

<pre class="prettyprint">
int fread(void *ptr, size_t size, size_t nmemb, FILE *fp);
int fwrite(const void *ptr, size_t size, size_t nmemb, FILE *fp);
</pre>

첫 번째 인자 `ptr`은 데이터를 위한 버퍼 공간이며, 두 번째 인자는 데이터의 크기, 세 번째 인자는 반복 횟수입니다.

<br>

## fseek() / rewind()

파일 스트림의 offset을 변경할 수 있는 함수들입니다.

<pre class="prettyprint">
int fseek(FILE *fp, long offset, int whence);
int rewind(FILE *fp);
</pre>

`fseek()` 함수는 [`lseek()` 함수와 사용법이 같습니다.](c++/2017/09/10/linux-control-file-as-low-level/)

`rewind()` 함수는 offset을 파일의 처음으로 이동시키며 `fseek(fp, 0, SEEK_SET)`와 같은 작업을 수행합니다.

<br>

## ftell() / fgetpos() / gsetpos()

<pre class="prettyprint">
long ftell(FILE *fp);
int fgetpos(FILE *fp, fpos_t *pos);
int fsetpos(FILE *fp, const fpos_t *pos);
</pre>

`ftell()` 함수는 파일의 현재 offset을 리턴하며, `fgetpos()` 함수는 현재 offset를 두 번째 인자인 `pos` 포인터로 리턴합니다. `fsetpos()` 함수는 파일의 offset을 두 번째 인자인 `pos` 값으로 이동시킵니다.

<br>

## fflush()

현재 버퍼에 있는 내용을 즉시 사용하며, 버퍼를 비우는 기능을 수행합니다.

<pre class="prettyprint">
int fflush(FILE *fp);
</pre>

함수가 성공적으로 수행하면 `0`을 리턴하고, 실패할 경우 `-1`을 리턴합니다. 만약 `fclose()` 함수를 수행하는 경우는 내부적으로 `flush` 작업을 수앻하기 때문에 별도로 `fflush()` 함수를 수행할 필요는 없습니다.