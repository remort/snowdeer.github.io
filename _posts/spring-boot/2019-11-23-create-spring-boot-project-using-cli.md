---
layout: post
title: CLI 이용해서 Spring Boot 프로젝트 생성하는 방법

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# 스프링 부트 프로젝트 시작하기

다음 명령어를 이용해서 간단한 스프링 부트 프로젝트를 생성할 수 있습니다.

<pre class="prettyprint">
$ spring init

Using service at https://start.spring.io
Content saved to 'demo.zip'
</pre>

명령어 실행 후 디렉토리를 보면 `demo.zip` 파일이 있으며 이 안에는 `maven` 기반의 `pom.xml` 빌드 명세가 들어 있는 
기본 프로젝트가 들어 있습니다.

프로젝트 구조는 다음과 같습니다.

~~~
.
├── HELP.md
├── mvnw
├── mvnw.cmd
├── pom.xml
└── src
    ├── main
    │   ├── java
    │   │   └── com
    │   │       └── example
    │   │           └── demo
    │   │               └── DemoApplication.java
    │   └── resources
    │       └── application.properties
    └── test
        └── java
            └── com
                └── example
                    └── demo
                        └── DemoApplicationTests.java

12 directories, 7 files
~~~

<br>

## 빌드 옵션 추가하기

다음과 같이 `spring init` 명령어 다음에 파라메터를 이용하여 커스텀 옵션을 추가할 수 있습니다.

<pre class="prettyprint">
$ spring init --d web,jpa,security --build gradle

Using service at https://start.spring.io
Content saved to 'demo.zip'
</pre>

빌드 타입을 `gradle`로 설정했으며, 데이터 영속성으로 `JPA`를 사용하고, Spring Security로 보안을 적용하는 예제입니다.

또한 `demo.zip` 압축 파일이 아니라 `snowdeer`라는 이름의 디렉토리에 `war` 형태의 프로젝트를 생성하려면 다음과 같은 명령어를 사용합니다.

<pre class="prettyprint">
spring init -dweb,jpa,security --build gradle -p war snowdeer

Using service at https://start.spring.io
Project extracted to '/Users/snowdeer/Workspace/SpringBoot/snowdeer'
</pre>

이 때 디렉토리 구조는 다음과 같습니다. `gradle` 기반이라 안드로이드 개발자들에겐 더 친숙한 구조 같습니다.

~~~
.
├── HELP.md
├── build.gradle
├── gradle
│   └── wrapper
│       ├── gradle-wrapper.jar
│       └── gradle-wrapper.properties
├── gradlew
├── gradlew.bat
├── settings.gradle
└── src
    ├── main
    │   ├── java
    │   │   └── com
    │   │       └── example
    │   │           └── snowdeer
    │   │               ├── DemoApplication.java
    │   │               └── ServletInitializer.java
    │   └── resources
    │       ├── application.properties
    │       ├── static
    │       └── templates
    └── test
        └── java
            └── com
                └── example
                    └── snowdeer
                        └── DemoApplicationTests.java

16 directories, 11 files
~~~

<br>

## 도움말

그 외에 `spring help init` 명령어나 `spring init --list` 명령어를 이용해서 도움말이나 지원하는 서비스 리스트를 확인할 수 있습니다.

<br>

## 실행

실행은 터미널에서 다음 명령어를 입력하면 됩니다.

<pre class="prettyprint">
gradle bootRun
</pre>

그런 다음 웹 브라우저에서 `http://127.0.0.1:8080`를 입력하면 페이지가 하나 출력이 될 것입니다.

만약 포트 번호를 바꾸려면 `application.properties` 파일을 수정하면 됩니다. 
이 파일은 `src/main/resources` 디렉토리 아래에 있으며 기본적으로 아무 것도 적혀 있지 않은 
빈 파일입니다.

아래와 같이 속성 값을 수정해서 기본 포트 번호를 `8080`에서 `8000`으로 변경할 수도 있습니다.

<pre class="prettyprint">
server.port=8000
</pre>
