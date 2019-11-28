---
layout: post
title: Spring Data JPA

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---
# Spring Data JPA

JPA(Java Persistence API)는 Java를 이용해서 데이터를 관리하는 기법을 스펙으로 정리한 표준입니다.
데이터베이스와 관련된 기술과 스펙은 오랫동안 이슈가 되어왔고, Java 진영에서는 EJB라는 기술 스펙으로
Entity Bean이라는 데이터 처리 스펙을 정했으며 이게 JPA의 시초라고 볼 수 있습니다.

<br>

## ORM

ORM(Object Relation Mapping)은 데이터베이스의 개체와 객체지향에서의 객체가 아주 유사하기 때문에
데이터베이스와 객체 지향을 한 번에 처리하기 위해서 나온 개념입니다.

ORM은 특정 언어에 종속적인 개념이 아니기 때문에 다양한 언어에서 ORM 지원 프레임워크를 많이 볼 수 있습니다.

<br>

## JPA와 ORM

JPA는 ORM의 개념을 Java에서 구현하기 위한 스펙이라고 볼 수 있습니다. 기존의 JDBC 등을 이용해서 
SQL 쿼리 등을 직접 작성했다면, ORM을 통해서 추상화된 객체 형태로 데이터베이스를 관리할 수 있습니다.

<br>

## 스프링 부트와 JPA

JPA는 하나의 스펙이기 때문에 다양한 프레임워크들에서 개별로 구현을 하고 있습니다. 스프링부트에서는
`Hibernate`를 이용해서 JPA를 지원하고 있습니다.

<br>

## JPA의 특징

JPA를 이용하면 다음과 같은 장점이 있습니다.

* 데이터베이스 관련 코드에 대한 유연함 획득
* DB 설계와 Java 설계를 동시에 할 수 있음
* 데이터베이스 종류와 독립적 - 특정 벤더에 종속적이지 않음

반대로 다음과 같은 단점도 있습니다.

* 높은 러닝 커브(Learning Curve)
* 객체지향 설계 사상이 반영되어야 함
* 특정 데이터베이스에 최적화된 방법을 사용할 수 없음