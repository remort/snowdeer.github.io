---
layout: post
title: atoi(), itoa() 함수 사용법
category: C++
tag: [C++]
---
# 문자열 변환 함수

`atoi()` 및 `itoa()` 함수는 문자형과 정수형간 데이터를 변한하는 함수입니다. 두 함수는 `stdlib.h`에 선언되어 있습니다.

<br>

## atoi()

`atoi()` 함수는 문자형을 정수형으로 변환합니다. 예를 들어 다음과 같은 코드 형태로 사용할 수 있습니다.

<pre class = "prettyprint">
int a = atoi("123");
</pre>

<br>

## itoa()

`itoa()`는 반대로 숫자를 문자형으로 변환하는 함수입니다. 다만 `itoa()`는 표준 함수는 아닙니다. 플랫폼에 따라 쓸 수 없기도 합니다. 하지만 숫자를 문자로 바꾸는 것은 대부분의 언어에서는 아주 쉬운 일입니다. C++에서는 `sprintf()` 함수 등을 이용해서 쉽게 변경할 수 있습니다.

<br>

## 그 외

그 외에도 문자열을 `double` 형태나 `long`, `long long` 형태로 바꿔주는 함수들도 존재합니다.


<pre class = "prettyprint">
double atof(const char* str);
long atol(const char* str);
long long atoll(const char* str);
</pre>