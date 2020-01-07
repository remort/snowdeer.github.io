---
layout: post
title: Kotlin Ktor 활용한 간단한 Login 페이지 만들기
category: Kotlin
tag: [Kotlin, ktor]
---

# Login 기능 구현하기

## login.ftl

`resources/templates/login.ftl` 파일을 생성합니다.

<pre class="prettyprint">
&lt;html&gt;
&lt;head&gt;
    &lt;link rel="stylesheet" href="/static/styles.css"&gt;
&lt;/head&gt;
&lt;body&gt;
&lt;#if error??&gt;
    &lt;p style="color:red;"&gt;${error}&lt;/p&gt;
&lt;/#if&gt;
&lt;form action="/login" method="post" enctype="application/x-www-form-urlencoded"&gt;
    &lt;div&gt;User:&lt;/div&gt;
    &lt;div&gt;&lt;input type="text" name="username" /&gt;&lt;/div&gt;
    &lt;div&gt;Password:&lt;/div&gt;
    &lt;div&gt;&lt;input type="password" name="password" /&gt;&lt;/div&gt;
    &lt;div&gt;&lt;input type="submit" value="Login" /&gt;&lt;/div&gt;
&lt;/form&gt;
&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

## Main.kt

<pre class="prettyprint">
package com.snowdeer

import freemarker.cache.ClassTemplateLoader
import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.freemarker.FreeMarker
import io.ktor.freemarker.FreeMarkerContent
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.request.receiveParameters
import io.ktor.response.respond
import io.ktor.response.respondText
import io.ktor.routing.get
import io.ktor.routing.post
import io.ktor.routing.route
import io.ktor.routing.routing

fun Application.main() {
    install(FreeMarker) {
        templateLoader = ClassTemplateLoader(this::class.java.classLoader, "templates")
    }

    routing {
        static("/static") {
            resources("static")
        }

        route("/login") {
            get {
                call.respond(FreeMarkerContent("login.ftl", null))
            }
            post {
                val post = call.receiveParameters()

                val username = post["username"]
                println("[snowdeer] username: $username")
                if (username != null && post["password"] != null) {
                    call.respondText("OK")
                } else {
                    call.respond(FreeMarkerContent("login.ftl", mapOf("error" to "Invalid login")))
                }
            }
        }
    }

}
</pre>

만약 리다이렉션(Redirection)을 하고 싶으면 다음과 같은 코드를 사용하면 됩니다.

<pre class="prettyprint">
call.respondRedirect("/", permanent = false)
</pre>