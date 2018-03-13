---
layout: post
title: 간단한 Web Framework 구현하기 (1) - 구조
category: Go
tag: [Go]
---
# Go언어 기반 간단한 Web Framework 구현

사용자로부터 요청(Request)이 오면 응답(Response)를 할 수 있는 간단한 Web Framework 예제코드입니다. 

참고로 한 책은 [Go언어 웹프로그래밍 철저입문](http://www.yes24.com/24/goods/25268498?scode=032&OzSrank=1)입니다.

<br>

## Web Framework 요구사항

Web Framework는 다음과 같은 기능을 제공해야 합니다.

* URI 패턴 맵핑 기능
* 로그 처리
* 에러 처리
* 정적 파일 처리
* 사용자 인증 및 권한 관리
* 보안 처리
* 세션 상태 관리
* 데이터베이스 관리
* 웹 요청 및 응답 추상화

<br>

## Web Framework 구조도

![Image](/assets/go/006.png)

각각의 요소들은 go 언어를 이용해서 하나씩 구현해봅니다.
