---
layout: post
title: Thymeleaf Sample 예제

category: Spring Boot
permalink: /spring-boot/:year/:month/:day/:title/
tag: [SpringBoot]
---

# Thymeleaf 라이브러리

타임리프(Thymeleaf) 라이브러리는 템플릿을 이용해서 `HTML` 페이지를 편리하게 만들 수 있도록 도와줍니다.
[Spring Initializer 웹사이트](https://start.spring.io)에서 'Thymeleaf' 항목을 선택하고 프로젝트를 생성하면 됩니다.

<br>

## SampleController.kt

<pre class="prettyprint">
@Controller
class SampleController {

    @GetMapping("/hello")
    fun hello(model: Model) {
        model.addAttribute("message", "Hello, SnowDeer.")
    }
}
</pre>

<br>

## hello.html

`resources/templates/` 디렉토리 아래에 `hello.html` 파일을 생성합니다.

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;
&lt;h2&gt;Thymeleaf Template Sample&lt;/h2&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>


그런 다음 웹브라우저에서 `http://localhost:8080/hello` 페이지에 접속하면 방금 생성한 `hello.html` 페이지가 뜨는 것을
확인할 수 있습니다.

그리고 위 코드를 아래처럼 수정을 하면,

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;
&lt;h2 th:text="${message}"&gt;Thymeleaf Template Sample&lt;/h2&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>

`SampleController`에서 `model.addAttribute` 메소드로 넘긴 파라메터가 잘 전송된 것을 확인할 수 있습니다.

<br>

## utext 명령어

`utext` 명령어는 HTML 문자열을 출력하는 명령어입니다.

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;
&lt;h2 th:text="${message}"&gt;Thymeleaf Template Sample&lt;/h2&gt;

th:text
&lt;div th:text='${"&lt;h3&gt;" + user.id + "&lt;/h3&gt;"}'/&gt;

&lt;br&gt;

th:utext
&lt;div th:utext='${"&lt;h3&gt;" + user.id + "&lt;/h3&gt;"}'/&gt;

&lt;/body&gt;
&lt;/html&gt;
</pre>

를 실행해보면 

~~~
th:text
<h3>snowdeer</h3>

th:utext
snowdeer
~~~

와 같이 출력되는 것을 확인할 수 있습니다.

<br>

## 리스트 출력하기

리스트를 위한 반복문은 `th:each` 명령어를 이용해서 처리할 수 있습니다.

<pre class="prettyprint">
@Controller
class SampleController {

    @GetMapping("/userList")
    fun userList(model: Model) {

        val list = ArrayList&lt;UserVO&gt;()
        for (i in 0 until 10) {
            val user = UserVO(uno = i.toLong(), id = "snowdeer$i", password = "1234", name = "Seo $i")

            list.add(user)
        }

        model.addAttribute("list", list)
    }
}
</pre>

<br>

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;

&lt;table border="1"&gt;
    &lt;tr&gt;
        &lt;td&gt;Id&lt;/td&gt;
        &lt;td&gt;Name&lt;/td&gt;
        &lt;td&gt;RegDate&lt;/td&gt;
    &lt;/tr&gt;

    &lt;tr th:each="user : ${list}"&gt;
        &lt;td th:text="${user.id}"/&gt;
        &lt;td th:text="${user.name}"/&gt;
        &lt;td th:text="${#dates.format(user.regDate, 'yyyy-MM-dd hh-mm-ss')}"/&gt;
    &lt;/tr&gt;

&lt;/table&gt;

&lt;/body&gt;
&lt;/html&gt;
</pre>

그리고 `th:each`에서 추가로 반복 상태에 대한 변수 값을 아래의 예제처럼 사용할 수도 있습니다.

<pre class="prettyprint">
    &lt;tr th:each="user, iter : ${list}"&gt;
        &lt;td th:text="${iter.index}"/&gt;
        &lt;td th:text="${iter.size}"/&gt;
        &lt;td th:text="${user.id}"/&gt;
        &lt;td th:text="${user.name}"/&gt;
        &lt;td th:text="${#dates.format(user.regDate, 'yyyy-MM-dd hh-mm-ss')}"/&gt;
    &lt;/tr&gt;
</pre>

그 외에도 `count`, `odd`, `even`, `first`, `last` 등의 값을 사용할 수도 있습니다.

<br>

## 특정 범위내 유효한 변수

특정 범위내에 유효한 변수는 `th:with`를 이용해서 구현할 수 있습니다. 예를 들면 다음과 같습니다.

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;

&lt;table border="1" th:with="target='snowdeer5'"&gt;
    &lt;tr&gt;
        &lt;td&gt;Id&lt;/td&gt;
        &lt;td&gt;Name&lt;/td&gt;
        &lt;td&gt;RegDate&lt;/td&gt;
    &lt;/tr&gt;

    &lt;tr th:each="user : ${list}"&gt;
        &lt;td th:text="${user.id == target ? 'SNOWDEER': user.id}"/&gt;
        &lt;td th:text="${user.name}"/&gt;
        &lt;td th:text="${#dates.format(user.regDate, 'yyyy-MM-dd hh-mm-ss')}"/&gt;
    &lt;/tr&gt;

&lt;/table&gt;

&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

## if, ~unless 구문

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;

&lt;table border="1" th:with="target='snowdeer5'"&gt;
    &lt;tr&gt;
        &lt;td&gt;Id&lt;/td&gt;
        &lt;td&gt;Name&lt;/td&gt;
        &lt;td&gt;RegDate&lt;/td&gt;
    &lt;/tr&gt;

    &lt;tr th:each="user : ${list}"&gt;
        &lt;td th:text="${user.id == target ? 'SNOWDEER': user.id}"/&gt;
         &lt;td th:if="${user.name}"&gt;
            &lt;a href="/profile" th:if="${user.name=='Seo 3'}"&gt;Seo 3 Profile&lt;/a&gt;
            &lt;p th:unless="${user.name=='Seo 3'}"&gt;Nothing&lt;/p&gt;
        &lt;/td&gt;
        &lt;td th:text="${#dates.format(user.regDate, 'yyyy-MM-dd hh-mm-ss')}"/&gt;
    &lt;/tr&gt;

&lt;/table&gt;

&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

## 자바스크립트에 연결하기

<pre class="prettyprint">
@Controller
class SampleController {

    @GetMapping("/useJavascript")
    fun useJavascript(model: Model) {
        val text = "Hello"

        model.addAttribute("text", text)

    }
}
</pre>

<br>

<pre class="prettyprint">
&lt;!DOCTYPE html&gt;
&lt;html xmlns:th="http://www.thymeleaf.org"&gt;
&lt;head&gt;
    &lt;meta charset="UTF-8"&gt;
    &lt;title&gt;Hello&lt;/title&gt;
&lt;/head&gt;
&lt;body&gt;

&lt;script th:inline="javascript"&gt;
    var text = [[${text}]]
    alert(text)
&lt;/script&gt;

&lt;/body&gt;
&lt;/html&gt;
</pre>