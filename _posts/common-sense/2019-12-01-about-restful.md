---
layout: post
title: REST 소개
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어]
---
# REST 사용방법

`REST`는 웹 등에서 사용하는 통신 네트워크 아키텍처입니다. 웹은 전송 방식으로 `HTTP`를 사용하며, 
식별 방법으로는 `URI`를 사용합니다. `REST`는 `HTTP`와 `URI`의 단순하고 간결한 장점을 활용한 네트워크 아키텍처입니다.
`REST`의 목적은 다음과 같습니다.

* 인터페이스의 범용성
* 개별 구성 요소의 독립적 배포
* 구성 요소의 확장성
* 중간적 구성 요소를 이용한 보안성, 응답 지연 감소, 레거시 시스템에 대한 캡슐화

<br>

### 일관된 인터페이스

인터페이스의 일관성에는 다음 요소가 있습니다.

* 자원 식별
* 메시지를 이용한 리소스 제어
* 자기 서술적 메시지
* HATEOAS(Hypermedia As The Engine Of Application State)

`HATEOAS`는 클라이언트에 응답을 할 때 단순한 결과 데이터만 리턴하는 것이 아닌, URI 정보를 함께 포함해야 한다는 원칙입니다.

`REST`의 제약 조건들을 지키면서 `REST` 아키텍처를 만드는 것을 `RESTful`이라고 합니다.

<br>

## URL과 URI

`URL`은 리소스를 가져오는 위치이며, `URI`는 리소스의 위치와 이름을 식별하는 표준입니다. 예를 들면 다음과 같습니다.

~~~
http://localhost:8080/book.pdf
~~~

`URL`은 위와 같이 리소스의 위치를 표현합니다. 그리고 `URI`는 아래와 같이 리소스의 이름과 위치를 식별합니다.

~~~
http://localhost:8080/api/books/100
~~~

`URL`은 `URI`의 하위 개념입니다. 

<br>

## URI 규격

`URI`는 명사를 사용하며 동사 사용은 자제하는 것이 좋습니다. 만약 동사를 표현하고 싶을 때는 `HTTP` 메소드인
`GET`, `POST`, `PUT`, `DELETE` 등으로 대체하는 것이 좋습니다.

`URI`에서는 명사에 단수형보다는 복수형을 사용하는 것이 좋습니다.