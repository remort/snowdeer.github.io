---
layout: post
title: Kotlin Simple Web Server (Ktor 활용)
category: Kotlin
tag: [Kotlin, ktor]
---

## build.gradle

<pre class="prettyprint">
group 'com.snowdeer'
version '1.0-SNAPSHOT'

buildscript {
    ext.kotlin_version = '1.3.61'
    ext.ktor_version = '1.2.6'

    repositories {
        mavenCentral()
    }
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

apply plugin: 'java'
apply plugin: 'kotlin'

sourceCompatibility = 1.8

repositories {
    mavenCentral()
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk8"
    compile "io.ktor:ktor-server-netty:$ktor_version"

    testCompile group: 'junit', name: 'junit', version: '4.12'
}

compileKotlin {
    kotlinOptions.jvmTarget = "1.8"
}
compileTestKotlin {
    kotlinOptions.jvmTarget = "1.8"
}

kotlin {
    experimental {
        coroutines "enable"
    }
}
</pre>

<br>

## BlogApp.kt

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.*
import io.ktor.http.*
import io.ktor.response.*
import io.ktor.routing.*
import io.ktor.server.engine.*
import io.ktor.server.netty.*

fun main(args: Array&lt;String&gt;) {
    embeddedServer(Netty, 8080) {
        routing {
            get("/") {
                call.respondText("SnowDeer's Blog", ContentType.Text.Html)
            }
        }
    }.start(wait = true)
}
</pre>

<br>

## Application 모듈을 활용한 BlogApp.kt

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.*
import io.ktor.features.*
import io.ktor.http.*
import io.ktor.response.*
import io.ktor.routing.*
import io.ktor.server.engine.*
import io.ktor.server.netty.*

fun Application.module() {
    install(DefaultHeaders)
    install(CallLogging)
    install(Routing) {
        get("/") {
            call.respondText("SnowDeer Blog2", ContentType.Text.Html)
        }
    }
}

fun main(args: Array&lt;String&gt;) {
    embeddedServer(Netty, 8080, watchPaths = listOf("BlogAppKt"), module = Application::module).start()
}
</pre>

<br>

## application.conf 사용해서 환경 변수 분리하기

`main/resources/` 디렉토리 아래에 `application.conf`에 파일을 만들고 다음 내용을 작성합니다.

<pre class="prettyprint">
ktor {
    deployment {
        port = 8070
    }

    application {
        modules = [ com.snowdeer.BlogAppKt.main ]
    }
}
</pre>

적용 확인을 위해서 포트 번호를 `8070`으로 했습니다.

그 이후 `BlogApp.kt` 파일은 다음과 같이 변경해줍니다.

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.*
import io.ktor.features.*
import io.ktor.http.*
import io.ktor.response.*
import io.ktor.routing.*

fun Application.main() {
    install(DefaultHeaders)
    install(CallLogging)
    install(Routing) {
        get("/") {
            call.respondText("SnowDeer Blog2", ContentType.Text.Html)
        }
    }
}
</pre>

기존 코드에서 `main` 전역 메소드가 사라졌고, `fun Application.module()` 메소드가 `fun Application.main()`로 변경되었습니다.

또한 `build.gradle`에 다음 내용을 추가합니다.

<pre class="prettyprint">
apply plugin: 'application'

mainClassName = "com.snowdeer.BlogAppKt"
</pre>

마지막으로 `IntelliJ` IDE에서 `Run -> Edit Configurations` 메뉴로 가서
`Main Class`를 `io.ktor.server.netty.EngineMain`으로 해주고 실행하면 위에서 작성한 `application.conf` 환경 변수가
적용되어 실행되는 것을 확인할 수 있습니다.