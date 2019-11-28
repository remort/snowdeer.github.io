---
layout: post
title: Application Properties를 이용한 프로파일 사용하기

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# 프로파일 사용하기

하나의 서비스 인터페이스와 이를 구현하는 서비스 구현체가 여러 개가 있을 때 프로파일을 이용해서 
원하는 서비스가 연결되도록 할 수 있습니다.

## HelloService.kt

<pre class="prettyprint">
interface HelloService {
    fun hello(name: String): String
}
</pre>

<br>

## HelloServiceImpl.kt

<pre class="prettyprint">
class HelloServiceImpl : HelloService {
    override fun hello(name: String): String {
        return "Hello, $name"
    }
}
</pre>

<br>

## GoodMorningServiceImpl.kt

<pre class="prettyprint">
class HelloServiceImpl : HelloService {
    override fun hello(name: String): String {
        return "Hello, $name"
    }
}
</pre>

기존에는 각 서비스 구현 클래스에 `@Service` 어노테이션을 붙였지만, 여기서는 붙이지 않았습니다. 
대신 Application 클래스안에 Bean 클래스를 명시적으로 생성하도록 합니다.

<br>

## HelloApplication.kt

<pre class="prettyprint">
@SpringBootApplication
class HelloApplication {
    @Bean
    fun helloService(): HelloService = HelloServiceImpl()

//    @Bean
//    fun goodMorinigServie(): HelloService = GoodMorningServiceImpl()
}

fun main(args: Array&lt;String&gt;) {
    runApplication&lt;HelloApplication&gt;(*args)
}
</pre>

위의 주석 부분을 제거하면 서비스가 2개가 되어 실행시 오류가 발생합니다.
이제 Application Properties를 이용해서 선택적으로 인스턴스를 생성하는 부분을 적용합니다.

<br>

## application.yaml

<pre class="prettyprint">
service:
  greet: "goodmorning"
</pre>

<br>

## HelloApplication.kt(수정 후)

<pre class="prettyprint">
@SpringBootApplication
class HelloApplication {
    @Bean
	@ConditionalOnExpression("#{'\${service.greet}'=='hello'}")
    fun helloService(): HelloService = HelloServiceImpl()

    @Bean
	@ConditionalOnExpression("#{'\${service.greet}'=='goodmorning'}")
    fun goodMorinigServie(): HelloService = GoodMorningServiceImpl()
}


fun main(args: Array&lt;String&gt;) {
    runApplication&lt;HelloApplication&gt;(*args)
}
</pre>