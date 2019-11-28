---
layout: post
title: Spring Boot 설치(Mac OS)

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# Spring Boot 설치 방법(Mac OS 기준)

여기서는 `brew`를 이용해서 설치하는 방법을 포스팅 합니다.

<pre class="prettyprint">
brew tap pivotal/tap

brew install springboot
</pre>

<br>

설치 후 다음 명령으로 스프링 부트 버전을 확인할 수 있습니다.

<pre class="prettyprint">
$ spring --version

Spring CLI v2.2.1.RELEASE
</pre>

그리고 `gradle`도 다음과 같이 설치할 수 있습니다.

<pre class="prettyprint">
brew install gradle
</pre>