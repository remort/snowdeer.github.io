---
layout: post
title: Error 처리하기

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# Error 처리하는 방법

## Json 파싱 에러 처리

`ErrorHandler.kt` 파일을 만들고 다음과 같이 작성하면 됩니다.

<pre class="prettyprint">
@ControllerAdvice
class ErrorHandler {

    @ExceptionHandler(JsonParseException::class)
    fun JsonParseExceptionHandler(servletRequest: HttpServletRequest, exception: Exception)
            : ResponseEntity&lt;String&gt; {
        return ResponseEntity("JSON Parsing Error", HttpStatus.BAD_REQUEST)
    }
}
</pre>

하지만 이 경우는 `String`으로 리턴하기 때문에 RESTful API에는 어울리지 않습니다. 
에러도 JSON 포맷으로 리턴하는 것이 좋습니다.

<br>

### ErrorResponse.kt

<pre class="prettyprint">
data class ErrorResponse(val error: String, val message: String)
</pre>

<br>

### ErrorHandler.kt

<pre class="prettyprint">
@ControllerAdvice
class ErrorHandler {

    @ExceptionHandler(JsonParseException::class)
    fun JsonParseExceptionHandler(servletRequest: HttpServletRequest, exception: Exception)
            : ResponseEntity&lt;ErrorResponse&gt; {
        return ResponseEntity(
                ErrorResponse("JSON Parsing Error", exception.message ?: ""),
                HttpStatus.BAD_REQUEST)
    }
}
</pre>

<br>

## 다양한 Exception 처리

여기에 좀 더 다양한 `Exception` 처리를 하려면 다음과 같이 추가로 예외 처리를 작성할 수 있습니다.

### ErrorResponse.kt

<pre class="prettyprint">
data class ErrorResponse(val error: String, val message: String)
</pre>

<br>

### UserNotFoundException.kt

<pre class="prettyprint">
class UserNotFoundException(message: String) : Exception(message)
</pre>

<br>

### ErrorHandler.kt

<pre class="prettyprint">
@ControllerAdvice
class ErrorHandler {

    @ExceptionHandler(JsonParseException::class)
    fun JsonParseExceptionHandler(servletRequest: HttpServletRequest, exception: Exception)
            : ResponseEntity&lt;ErrorResponse&gt; {
        return ResponseEntity(
                ErrorResponse("JSON Parsing Error", exception.message ?: ""),
                HttpStatus.BAD_REQUEST)
    }

    @ExceptionHandler(UserNotFoundException::class)
    fun UserNotFoundExceptionHandler(servletRequest: HttpServletRequest, exception: Exception)
            : ResponseEntity&lt;ErrorResponse&gt; {
        return ResponseEntity(
                ErrorResponse("User Not Found", exception.message ?: ""),
                HttpStatus.NOT_FOUND)
    }
}
</pre>

<br>


### UserController.kt

<pre class="prettyprint">
@RestController
class UserController {

    @Autowired
    private lateinit var service: UserService

    @GetMapping("/user/{id}")
    fun getUser(@PathVariable id: Int): ResponseEntity<User> {
        val user = service.getUser(id) ?: throw UserNotFoundException("user($id) does not exist.")
        return ResponseEntity(user, HttpStatus.OK)
    }
}
</pre>