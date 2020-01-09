---
layout: post
title: Kotlin Ktor start.io 사용해서 프로젝트 생성하기
category: Kotlin
tag: [Kotlin, ktor]
---

# Ktor start.io

`Spring Boot`같은 경우는 [Spring Initializer](https://start.spring.io)라는 사이트를 이용해서 
초기 프로젝트를 손쉽게 생성할 수 있습니다.

`Ktor`도 비슷한 역할을 해주는 사이트가 있습니다.

* [Ktor Project Generator](https://start.ktor.io/)

이런 사이트를 이용해서 프로젝트를 생성할 시 좋은 점은 원하는 라이브러리를 찾기가 쉬우며, 
각 라이브러리간 버전 차이로 발생하는 문제를 최소화할 수 있기 때문에 가급적 사용하는 것을 추천합니다.

위 사이트를 통해서 만들어진 `build.gradle` 예제입니다.

<br>

## build.gradle

<pre class="prettyprint">
buildscript {
    repositories {
        jcenter()
    }
    
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

apply plugin: 'kotlin'
apply plugin: 'application'

group 'com.snowdeer'
version '0.0.1-SNAPSHOT'
mainClassName = "io.ktor.server.netty.EngineMain"

sourceSets {
    main.kotlin.srcDirs = main.java.srcDirs = ['src']
    test.kotlin.srcDirs = test.java.srcDirs = ['test']
    main.resources.srcDirs = ['resources']
    test.resources.srcDirs = ['testresources']
}

repositories {
    mavenLocal()
    jcenter()
    maven { url 'https://kotlin.bintray.com/ktor' }
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk8:$kotlin_version"
    implementation "io.ktor:ktor-server-netty:$ktor_version"
    implementation "ch.qos.logback:logback-classic:$logback_version"
    implementation "io.ktor:ktor-server-core:$ktor_version"
    implementation "io.ktor:ktor-server-host-common:$ktor_version"
    implementation "io.ktor:ktor-gson:$ktor_version"
    testImplementation "io.ktor:ktor-server-tests:$ktor_version"
}
</pre>

<br>

## resources/application.conf

<pre class="prettyprint">
ktor {
    deployment {
        port = 8080
        port = ${?PORT}
    }
    application {
        modules = [ com.snowdeer.ApplicationKt.module ]
    }
}
</pre>

<br>

## resources/logback.xml

<pre class="prettyprint">
&lt;configuration&gt;
    &lt;appender name="STDOUT" class="ch.qos.logback.core.ConsoleAppender"&gt;
        &lt;encoder&gt;
            &lt;pattern&gt;%d{YYYY-MM-dd HH:mm:ss.SSS} [%thread] %-5level %logger{36} - %msg%n&lt;/pattern&gt;
        &lt;/encoder&gt;
    &lt;/appender&gt;

    &lt;root level="trace"&gt;
        &lt;appender-ref ref="STDOUT"/&gt;
    &lt;/root&gt;

    &lt;logger name="org.eclipse.jetty" level="INFO"/&gt;
    &lt;logger name="io.netty" level="INFO"/&gt;
&lt;/configuration&gt;
</pre>

<br>

## Application.kt

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.*
import io.ktor.response.*
import io.ktor.request.*
import io.ktor.routing.*
import io.ktor.http.*
import io.ktor.http.content.*
import io.ktor.gson.*
import io.ktor.features.*
import org.slf4j.event.*

fun main(args: Array&lt;String&gt;): Unit = io.ktor.server.netty.EngineMain.main(args)

@Suppress("unused") // Referenced in application.conf
@kotlin.jvm.JvmOverloads
fun Application.module(testing: Boolean = false) {
    install(ContentNegotiation) {
        gson {
        }
    }

    install(CallLogging) {
        level = Level.INFO
        filter { call -> call.request.path().startsWith("/") }
    }

    install(CORS) {
        method(HttpMethod.Options)
        method(HttpMethod.Put)
        method(HttpMethod.Delete)
        method(HttpMethod.Patch)
        header(HttpHeaders.Authorization)
        header("MyCustomHeader")
        allowCredentials = true
        anyHost() // @TODO: Don't do this in production if possible. Try to limit it.
    }

    routing {
        get("/") {
            call.respondText("HELLO WORLD!", contentType = ContentType.Text.Plain)
        }

        // Static feature. Try to access `/static/ktor_logo.svg`
        static("/static") {
            resources("static")
        }

        get("/json/gson") {
            call.respond(mapOf("hello" to "world"))
        }
    }
}
</pre>