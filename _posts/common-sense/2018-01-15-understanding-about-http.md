---
layout: post
title: HTTP에 대한 이해
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어]
---
# HTTP에 대한 이해

HTTP는 월드 와이드 웹(World Wide Web)에 대한 어플리케이션 레벨의 통신 프로토콜입니다. 텍스트 기반으로 되어 있지만 강력합니다.

HTTP는 FTP나 Telnet처럼 연결 지향 프로토콜(Connection-Oriented Protocol)이 아닌, 상태가 없는 프로토콜입니다. 동일 서버에 대해 요청을 여러 번 했을 때, 각 요청들은 이전 요청들에 대한 기록을 전혀 알 수 없습니다. 연결 지향성 프로토콜의 경우는 끊임없이 서로간의 메세지를 체크하기 위해 연결을 계속 유지시켜야 하지만, HTTP는 그렇지 않습니다.

HTTP는 원래 HTML만 전송하도록 설계되었습니다. HTTP 0.9 버전에서는 `GET` 메소드만 지원했습니다.

<br>

## HTTP Request

HTTP는 요청(Request)과 응답(Response) 프로토콜입니다. HTTP 요청은 다음과 같은 구성으로 이루어져 있습니다.

1. 요청
2. 빈 라인 또는 요청 헤더
3. 빈 라인
4. 메시지 본문

예를 들면 다음과 같은 형태로 이루어져 있습니다.

~~~
GET /blog/categories/ HTTP/1.1
HOST: snowdeer.github.io
User-Agent: Mozilla/5.0
(Empty Line)
~~~

위 예제에서 첫 번째 라인이 요청 메소드이며, 그 다음은 URI(Uniform Resource Identifier)입니다. 그리고 마지막으로 HTTP 버전을 명시합니다.

두 번째 라인과 세 번째 라인은 요청 헤더입니다. 마지막 라인의 경우 비어있는데 메시지 본문이 없더라도 반드시 포함되어야 합니다.

<br>

## HTML 지원 메소드

`GET` 메소드는 HTTP의 기본적인 요청입니다. `POST`의 경우 HTML 2.0부터 지원하기 시작했습니다. HTML은 `GET`과 `POST` 이 외의 메소드는 별도로 지원하지 않습니다. HTML 5의 초안에서 `PUT`과 `DELETE` 메소드를 추가로 지원하기로 했었으나 마지막에는 지원하지 않기로 결정했습니다.

<br>

## HTTP Response

HTTP Response는 서버가 클라이언트에게 전달하는 메시지이며 다음과 같은 구성으로 이루어져 있습니다.

1. 응답 상태
2. 0개 혹은 그 이상의 응답 헤더
3. 빈 라인
4. 메시지 본문

HTTP 응답 상태는 다음과 같이 5 종류로 이루어져 있습니다.

* 1xx : 서버가 요청을 받아 이미 처리중임
* 2xx : 요청을 성공적으로 수행했음. 보통 `200 OK`로 리턴
* 3xx : 리다이렉션(Redirection)
* 4xx : 페이지를 찾을 수 없는 경우 발생. 보통 `404 Not Found`로 리턴
* 5xx : 서버에 문제 발생했음을 의미


<br>

## HTTP/2

HTTP/2는 속도에 중점을 두고 있으며 SPDY/2에 기반하고 있습니다. HTTP/1.x는 텍스트 기반의 프로토콜이었으나 HTTP/2는 바이너리(Binary) 프로토콜입니다. HTTP/1.x에 비해 디버깅이 어려운 단점은 있습니다.

단일 요청을 수행했던 HTTP/1.x와 달리 HTTP/2는 완전 다중화(Fully multiplexed)로 되어 있습니다. 같은 시간대 여러 연결을 허용하고 다중 요청과 다중 응답을 해줄 수 있습니다. 또한 HTTP/2는 헤더를 압축해서 과부화를 줄이며, 서버측에서 클라이언트로 푸시(Push) 응답을 보내는 것도 허용합니다.