---
layout: post
title: Kotlin Ktor의 DSL을 이용한 웹페이지 생성
category: Kotlin
tag: [Kotlin, ktor]
---

## DSL을 이용한 웹페이지 만들기

`FreeMaker` 모듈 대신 HTML DSL(Domain specific language)를 이용해서 웹페이지를 
응답하는 코드 예제입니다.

보다 자세한 내용은 [여기](https://ktor.io/servers/features/templates/html-dsl.html)에서 볼 수 있습니다.

## build.gradle

Ktor용 App을 위한 기본 `build.gradle`의 종속성에 `FreeMaker` 종속성 대신에
 `implementation "io.ktor:ktor-html-builder:$ktor_version"` 항목이 추가되었습니다.

`repositories` 항목에 `jcenter()`를 추가해야 됩니다.

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
apply plugin: 'application'

sourceCompatibility = 1.8
application {
    mainClassName = "com.snowdeer.BlogAppKt"
}

repositories {
    jcenter()
    mavenCentral()
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk8"

    implementation "io.ktor:ktor-html-builder:$ktor_version"
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

import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.html.respondHtml
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.routing.get
import io.ktor.routing.routing
import kotlinx.html.*

data class IndexData(val items: List&lt;Int&gt;)

fun Application.main() {
    routing {
        static("/static") {
            resources("static")
        }

        get("/") {
            val data = IndexData(listOf(1, 2, 3))
            call.respondHtml {
                head {
                    link(rel = "stylesheet", href = "/static/styles.css")
                }
                body {
                    ul {
                        for (item in data.items) {
                            li { +"$item" }
                        }
                    }
                }
            }
        }
    }
}

</pre>