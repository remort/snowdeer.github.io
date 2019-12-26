---
layout: post
title: Property 읽어오기

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# yaml 파일 생성

## application.yaml

`resources` 디렉토리 아래에 `application.yaml` 파일을 생성하고 다음 내용을 입력합니다.

<pre class="prettyprint">
property:
  snowdeer:
    age: 40
    name: snowdeer
    address: Seoul
</pre>

<br>

# 값 읽어오기

Controller 클래스에서 값을 바로 읽어들여도 상관없지만, 여기서는 Wrapping 클래스를 별도로 생성했습니다.

## AppProperties.kt

<pre class="prettyprint">
import org.springframework.beans.factory.annotation.Value
import org.springframework.stereotype.Component

@Component
class AppProperties {

    @Value("\${property.snowdeer.age}")
    lateinit var age: Integer

    @Value("\${property.snowdeer.name}")
    lateinit var name: String

    @Value("\${property.snowdeer.address}")
    lateinit var address: String

}
</pre>

<br>

## HelloController.kt

<pre class="prettyprint">
import com.snowdeer.sample.board.AppProperties
import org.springframework.beans.factory.annotation.Autowired
import org.springframework.web.bind.annotation.GetMapping
import org.springframework.web.bind.annotation.RestController

@RestController
class HelloController {

    @Autowired
    lateinit var appProperties: AppProperties

    @GetMapping
    fun hello(): String {
        return "Hello ${appProperties.name}"
    }
}
</pre>