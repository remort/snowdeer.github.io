---
layout: post
title: 의존 관계 자동 설정

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# 의존 관계 자동 설정

앞서 포스팅한 `PathVariable 간단한 예제`의 예제 코드는 다음과 같습니다.

<br>

## SimpleService.kt

<pre class="prettyprint">
@Service
class SimpleService {
    fun hello(name: String): String {
        return "hello $name"
    }
}
</pre>

<br>

## SimpleController.kt

<pre class="prettyprint">
@Controller
class SimpleController(val simpleService: SimpleService) {

    @RequestMapping(value = ["/user/{name}"], method = arrayOf(RequestMethod.GET))
    @ResponseBody
    fun hello(@PathVariable name: String): String {
        return simpleService.hello(name)
    }
}
</pre>

`SimpleController` 클래스에서 `SimpleService` 인스턴스를 클래스 생성자의 매개변수로 전달하고 있습니다.
만약 해당 인자를 생성자의 매개 변수를 전달하고 싶지 않을 때는 다음과 같이 `@Autowired` 어노테이션을 이용할 수 있습니다.

<pre class="prettyprint">
@Controller
class SimpleController() {

    @Autowired
    private lateinit var service: SimpleService

    @RequestMapping(value = ["/user/{name}"], method = arrayOf(RequestMethod.GET))
    @ResponseBody
    fun hello(@PathVariable name: String): String {
        return service.hello(name)
    }
}
</pre>

이와 같이 사용하는 것을 의존성 삽입(Dependency Injection)이라고 합니다.

<br>

## 인터페이스와 구현체 분리

위에서 만들었던 `SimpleService` 클래스를 다음과 같이 인터페이스와 구현체로 분리할 수 있습니다.

<pre class="prettyprint">
interface SimpleService {
    fun hello(name: String): String
}
</pre>

<pre class="prettyprint">
@Service
class SimpleServiceImpl : SimpleService {
    override fun hello(name: String): String {
        return "hello $name"
    }
}
</pre>

그리고 `SimpleController`도 다음과 같이 수정합니다.

<pre class="prettyprint">
@Controller
class SimpleController() {

    @Autowired
    private lateinit var service: SimpleService

    @RequestMapping(value = ["/user/{name}"], method = arrayOf(RequestMethod.GET))
    @ResponseBody
    fun hello(@PathVariable name: String): String {
        return service.hello(name)
    }
}
</pre>

위 `SimpleController` 코드 안에는 `SimpleServiceImpl`과 관련된 코드가 하나도 없습니다. 
그런데도 스프링 부트가 `SimpleService` 인터페이스를 구현하는 적절한 인스턴스를 연결해서 매핑하는 것을 확인할 수 있습니다.
만약 `SimpleService` 인터페이스를 구현한 `SimpleServiceImpl2` 클래스를 추가로 생성하면 
실행할 때 다음과 같은 오류가 발생합니다.

~~~
Field service in com.snowdeer.board.controller.SimpleController required a single bean, but 2 were found:
	- simpleServiceImpl: defined in file [/Users/snowdeer/Workspace/SpringBoot/board/build/classes/kotlin/main/com/snowdeer/board/service/SimpleServiceImpl.class]
	- simpleServiceImpl2: defined in file [/Users/snowdeer/Workspace/SpringBoot/board/build/classes/kotlin/main/com/snowdeer/board/service/SimpleServiceImpl2.class]
~~~

<br>

## 의존 관계 자동 설정을 통한 장점

이와 같이 의존 관계 자동 설정을 이용하면 다음과 같은 장점을 얻을 수 있습니다.

* 세부 구현 정보 숨기기: 서비스 구현체에 종속성이 없더라도 해당 인스턴스를 사용가능하며, 따라서 서비스 작동 방식을 외부에 노출하지 않음
* 디커플링: 기존 서비스에 영향을 주지 않고, 새로운 서비스 변경이나 구현의 사용이 가능
* 쉬운 변경 처리: 스프링 설정을 통해 코드 수정 없이 서비스 변경이 가능함

