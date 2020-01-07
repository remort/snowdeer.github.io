---
layout: post
title: Kotlin Ktor 활용한 간단한 웹페이지(index.html) 띄우기 (FreeMarker 활용)
category: Kotlin
tag: [Kotlin, ktor]
---

## build.gradle

Ktor용 App을 위한 기본 `build.gradle`의 종속성에 `implementation "io.ktor:ktor-freemarker:$ktor_version"` 항목이
추가되었습니다.

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
    mavenCentral()
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk8"

    implementation "io.ktor:ktor-freemarker:$ktor_version"
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

## index.ftl

`resources/templates/index.ftl`에 템플릿 파일을 추가합니다.

<pre class="prettyprint">
&lt;html&gt;
	&lt;body&gt;
		&lt;ul&gt;
		&lt;#list data.items as item&gt;
			&lt;li&gt;${item}&lt;/li&gt;
		&lt;/#list&gt;
		&lt;/ul&gt;
	&lt;/body&gt;
&lt;/html&gt;
</pre>

<br>

## BlogApp.kt

<pre class="prettyprint">
package com.snowdeer

import freemarker.cache.ClassTemplateLoader
import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.freemarker.FreeMarker
import io.ktor.freemarker.FreeMarkerContent
import io.ktor.response.respond
import io.ktor.routing.get
import io.ktor.routing.routing

data class IndexData(val items: List&lt;Int&gt;)

fun Application.main() {
    install(FreeMarker) {
        templateLoader = ClassTemplateLoader(this::class.java.classLoader, "templates")
    }

    routing {
        get("/html-freemarker") {
            call.respond(FreeMarkerContent("index.ftl", mapOf("data" to IndexData(listOf(1, 2, 3))), ""))
        }
    }
}
</pre>