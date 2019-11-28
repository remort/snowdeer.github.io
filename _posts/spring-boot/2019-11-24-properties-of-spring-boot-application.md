---
layout: post
title: Application Properties 설정 방법

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# 어플리케이션 설정 값 정의 방법

스프링 부트는 어플리케이션 설정 값을 정의하기 위해 다음과 같은 방법을 제공하고 있습니다.

* application.properties 파일 이용
* yaml 파일
* CLI 명령어 매개변수

<br>

## application.properties

<pre class="prettyprint">
SERVICE.MESSAGE.TEXT="hello"
</pre>

<br>

## application.yaml

<pre class="prettyprint">
service:
    message:
        text: "hello"
</pre>

좀 더 복잡한 설정을 사용할 때는 `yaml` 파일을 이용하는 것이 가독성이나 관리가 더 쉽습니다.

<br>

## CLI 매개 변수

<pre class="prettyprint">
java jar ./snowdeer.jar --service.message.text="hello"
</pre>