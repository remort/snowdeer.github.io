---
layout: post
title: 간단한 Restful Server 만들기

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# Simple Restful Server

스프링부트에서 간단히 `@RestController` 어노테이션이 붙은 클래스를 추가하는 것만으로 
간단한 Restful 서버가 만들어집니다.

그리고 `data class` 클래스를 만들어서 해당 인스턴스를 반환하면 자동으로 `JSON` 포맷으로 전송됩니다.

간단한 예제는 다음과 같습니다.

<br>

## User.kt

<pre class="prettyprint">
data class MobileDevice(var id: Int = 0, var name: String = "")
</pre>

<br>

## UserController.kt

<pre class="prettyprint">
@RestController
class UserController {

    @RequestMapping("/user")
    fun user(): User {
        return User(1, "snowdeer")
    }
}
</pre>

<br>

이 상태에서 `http://localhost:8080/user` 주소로 접속하면 다음과 같은 결과가 출력됩니다.

~~~
{"id":1,"name":"snowdeer"}
~~~

<br>

## 경로 및 매개변수

이제 URL에 추가 매개변수를 넣어 필터링을 할 수가 있습니다.

매개변수에 따라 필터링되는 결과를 확인하기 위해, `UserApplication.kt` 파일에 다음과 같은 리스트를 추가합니다.

<br>

### UserApplication.kt

<pre class="prettyprint">
@SpringBootApplication
class UserApplication {
    companion object {
        val userList = ConcurrentHashMap&lt;Int, User&gt;()
    }

    init {
        userList[1] = User(1, "snowdeer")
        userList[2] = User(2, "ran")
        userList[3] = User(3, "yang")
        userList[4] = User(4, "downy")
        userList[5] = User(5, "john")
    }

    @Bean
    fun userList(): ConcurrentHashMap&lt;Int, User&gt; {
        return userList
    }
}

fun main(args: Array&lt;String&gt;) {
    runApplication&lt;UserApplication&gt;(*args)
}
</pre>

<br>

### UserController.kt

<pre class="prettyprint">
@RestController
class UserController {

    @Autowired
    private lateinit var userList: ConcurrentHashMap&lt;Int, User&gt;

    @RequestMapping("/user/{id}")
    fun user(@PathVariable id: Int): User? {
        return userList[id]
    }

    @RequestMapping("/users")
    fun userList(): List&lt;User&gt; {
        return userList.map(Map.Entry&lt;Int, User&gt;::value).toList()
    }
}
</pre>

<br>

## 검색어를 이용한 필터링

`UserController` 클래스에 `search()` 메소드를 추가합니다.

<pre class="prettyprint">
@RestController
class UserController {

    @Autowired
    private lateinit var userList: ConcurrentHashMap&lt;Int, User&gt;

    @RequestMapping("/user/{id}")
    fun user(@PathVariable id: Int): User? {
        return userList[id]
    }

    @RequestMapping("/users")
    fun userList(): List&lt;User&gt; {
        return userList.map(Map.Entry&lt;Int, User&gt;::value).toList()
    }

    @RequestMapping("/search")
    fun search(@RequestParam(required = false, defaultValue = "") nameFilter: String): List<User> {
        return userList.filter {
            it.value.name.contains(nameFilter, true)
        }.map(Map.Entry<Int, User>::value).toList()
    }
}
</pre>

그런 다음, `http://localhost:8080/find?nameFilter=an`과 같이 요청을 날리면 다음처럼 결과가 나옵니다.

~~~
[{"id":2,"name":"ran"},{"id":3,"name":"yang"}]
~~~