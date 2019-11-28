---
layout: post
title: PathVariable 간단한 예제 

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# PathVariable 간단한 예제 

샘플 프로젝트를 생성하고 다음 파일들을 생성합니다.

<br>

## SimpleService.kt

<pre class="prettyprint">
import org.springframework.stereotype.Service

@Service
class SimpleService {
    fun hello(name: String): String {
        return "hello $name"
    }
}
</pre>

위 클래스는 아직 큰 의미는 없고, 단순 메소드를 제공하는 클래스입니다.

<br>

## SimpleController.kt

<pre class="prettyprint">
import com.snowdeer.board.service.SimpleService
import org.springframework.stereotype.Controller
import org.springframework.web.bind.annotation.PathVariable
import org.springframework.web.bind.annotation.RequestMapping
import org.springframework.web.bind.annotation.RequestMethod
import org.springframework.web.bind.annotation.ResponseBody

@Controller
class SimpleController(val simpleService: SimpleService) {

    @RequestMapping(value = ["/user/{name}"], method = arrayOf(RequestMethod.GET))
    @ResponseBody
    fun hello(@PathVariable name: String): String {
        return simpleService.hello(name)
    }
}
</pre>

위와 같이 코드를 만들면, `/user/name`에 해당하는 URL의 `name` 부분을 경로 파라메터로 받을 수 있습니다.