---
layout: post
title: Kotlin Ktor를 활용한 HTTP API 서버 만들기
category: Kotlin
tag: [Kotlin, ktor]
---

# Ktor를 활용한 HTTP API 서버 구현

## 간단한 라우팅 구현

<pre class="prettyprint">
fun Application.main() {

    routing {
        static("/static") {
            resources("static")
        }

        routing {
            get("/apis") {
                call.respondText("OK")
            }
        }
    }
}
</pre>

위 코드로 `http://localhost:8080/apis` 경로로 접속하면 `OK`라는 텍스트를 리턴하는 웹서버를 만들 수 있습니다.

<br>

## Jakson 라이브러리 추가

HTTP API들은 보통 JSON 형태의 데이터를 많이 사용합니다. `jakson` 라이브러리를 사용하면 보다 쉽게 JSON 데이터를 
전송할 수 있습니다.

`build.gradle`에 다음 종속성을 추가합니다.

<pre class="prettyprint">
implementation "io.ktor:ktor-jackson:$ktor_version"
</pre>

그리고 위의 `Application.main()` 코드를 다음과 같이 변경합니다.

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.features.ContentNegotiation
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.jackson.jackson
import io.ktor.response.respond
import io.ktor.routing.get
import io.ktor.routing.routing

fun Application.main() {

    install(ContentNegotiation) {
        jackson {
        }
    }

    routing {
        static("/static") {
            resources("static")
        }

        routing {
            get("/apis") {
                call.respond(mapOf("id" to "snowdeer"))
            }
        }
    }
}
</pre>

만약 복수의 데이터를 Map 형태로 리턴하고 싶으면 다음과 같이 작성할 수 있습니다.

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.features.ContentNegotiation
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.jackson.jackson
import io.ktor.response.respond
import io.ktor.routing.get
import io.ktor.routing.routing
import java.util.*

fun Application.main() {

    install(ContentNegotiation) {
        jackson {
        }
    }

    routing {
        static("/static") {
            resources("static")
        }

        routing {
            get("/apis") {

                val map = HashMap&lt;String, Any&gt;()
                map["id"] = "snowdeer"
                map["age"] = 40
                map["email"] = "snowdeer0314@gmail.com"
                map["boolean"] = true
                map["float"] = 3.14F
                map["long"] = 30L

                call.respond(mapOf("data" to map))
            }
        }
    }
}
</pre>

이 때 `http://localhost:8080/apis`로 리퀘스트를 날리면 응답은 다음과 같습니다.

~~~
{
    "data": {
        "boolean": true,
        "id": "snowdeer",
        "float": 3.14,
        "age": 40,
        "email": "snowdeer0314@gmail.com",
        "long": 30
    }
}
~~~

<br>

## HTTP POST를 이용한 데이터 수신

`POST`를 이용해서 JSON 데이터를 수신받을 때는 `call.receive<Type>()` 메소드를 이용해서 전달 받을 수 있습니다.

<pre class="prettyprint">
package com.snowdeer

import com.fasterxml.jackson.databind.SerializationFeature
import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.features.ContentNegotiation
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.jackson.jackson
import io.ktor.request.receive
import io.ktor.response.respond
import io.ktor.routing.get
import io.ktor.routing.post
import io.ktor.routing.routing
import java.util.*
import kotlin.collections.ArrayList

data class PostItem(val id: String, val email: String)

fun Application.main() {

    val list = ArrayList&lt;PostItem&gt;()

    install(ContentNegotiation) {
        jackson {
            enable(SerializationFeature.INDENT_OUTPUT)
        }
    }

    routing {
        static("/static") {
            resources("static")
        }

        routing {
            get("/apis") {

                val map = HashMap&lt;String, Any&gt;()
                map["id"] = "snowdeer"
                map["age"] = 40
                map["email"] = "snowdeer0314@gmail.com"
                map["boolean"] = true
                map["float"] = 3.14F
                map["long"] = 30L

                call.respond(mapOf("data" to map))
            }

            post("/apis/add") {
                val data = call.receive&lt;PostItem&gt;()
                println("[snowdeer] receive: ${data}")

                list.add(data)

                call.respond(mapOf("result" to true))
            }

            get("/apis/list") {
                call.respond(mapOf("data" to list))
            }
        }
    }
}
</pre>

POST로 데이터를 전송할 때 주의할 점은, Request의 Key/Value로 값을 전달하는게 아니라
HTTP Body에 데이터를 문자열 형태로 전송해야 한다는 점입니다.

예를 들면 다음과 같은 형태로 HTTP Request를 전송해야 합니다.

~~~
POST http://127.0.0.1:8080/apis/add
Content-Type: application/json

{
	"id": "snowdeer",
	"email" : "hello"
}
~~~

만약 JSON 포맷이 다르거나 잘못된 내용을 전송할 경우 `500Internal Server Error`를 리턴합니다.