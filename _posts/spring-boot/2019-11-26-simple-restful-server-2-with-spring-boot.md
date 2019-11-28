---
layout: post
title: 간단한 Restful Server 만들기(리팩토링 버전)

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# Simple Restful Server

기존에 `UserApplication.kt` 내부에서 만들었던 User 리스트 정보를 `UserService` 클래스로 이동시킵니다.
그리고 `UserService` 클래스를 직접 참조하는게 아니라 인터페이스를 통해 접근하기 위해서,
`IUser` 인터페이스와 `UserService` 클래스로 분리를 합니다.

<br>

## UserApplication.kt

<pre class="prettyprint">
@SpringBootApplication
class UserApplication

fun main(args: Array&lt;String&gt;) {
    runApplication&lt;UserApplication&gt;(*args)
}
</pre>

<br>

## IUser.kt

<pre class="prettyprint">
interface IUser {
    fun getUser(id: Int): User?
    fun searchUser(nameFilter: String): List&lt;User&gt;
    fun createUser(user: User)
    fun updateUser(id: Int, user: User)
    fun deleteUser(id: Int)
}
</pre>

<br>

## UserService.kt

<pre class="prettyprint">
@Service
class UserService : IUser {
    private val userMap = ConcurrentHashMap&lt;Int, User&gt;()

    init {
        userMap[1] = User(1, "snowdeer")
        userMap[2] = User(2, "ran")
    }

    override fun getUser(id: Int): User? {
        return userMap[id]
    }

    override fun searchUser(nameFilter: String): List&lt;User&gt; {
        return userMap.filter {
            it.value.name.contains(nameFilter, true)
        }.map(Map.Entry&lt;Int, User&gt;::value).toList()
    }

    override fun createUser(user: User) {
        userMap[user.id] = user
    }

    override fun updateUser(id: Int, user: User) {
        userMap[id] = user
    }

    override fun deleteUser(id: Int) {
        userMap.remove(id)
    }
}
</pre>

<br>

## UserController.kt

<pre class="prettyprint">
@RestController
class UserController {

    @Autowired
    private lateinit var service: UserService

    @GetMapping("/user/{id}")
    fun getUser(@PathVariable id: Int): ResponseEntity&lt;User&gt; {
        return ResponseEntity(service.getUser(id), HttpStatus.OK)
    }

    @GetMapping("/users")
    fun userList(): ResponseEntity&lt;List&lt;User&gt;&gt; {
        return ResponseEntity(service.searchUser(""), HttpStatus.OK)
    }

    @GetMapping("/search")
    fun search(@RequestParam(required = false, defaultValue = "") nameFilter: String): ResponseEntity&lt;List&lt;User&gt;&gt; {
        return ResponseEntity(service.searchUser(nameFilter), HttpStatus.OK)
    }

    @PostMapping("/create")
    fun createUser(@RequestBody user: User): ResponseEntity&lt;Unit&gt; {
        service.createUser(user)
        return ResponseEntity(Unit, HttpStatus.OK)
    }

    @DeleteMapping("/delete/{id}")
    fun deleteUser(@PathVariable id: Int): ResponseEntity&lt;Unit&gt; {
        service.deleteUser(id)
        return ResponseEntity(Unit, HttpStatus.OK)
    }

    @PutMapping("/update/{id}")
    fun updateUser(@PathVariable id: Int, @RequestBody user: User): ResponseEntity&lt;Unit&gt; {
        service.updateUser(id, user)
        return ResponseEntity(Unit, HttpStatus.OK)
    }
}
</pre>

<br>

## POST 함수 테스트하기

터미널에서 `curl` 명령어를 이용해서 간단하게 테스트할 수 있습니다.

<pre class="prettyprint">
curl -X POST "http://localhost:8080/create" \
    -H "content-type: application/json" \
    -d '{ 
        "id": 3, 
        "name": "yang" 
        }'
</pre>