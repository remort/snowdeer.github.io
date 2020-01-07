---
layout: post
title: Kotlin Ktor 활용한 간단한 웹페이지(index.html)에 CSS 적용하기
category: Kotlin
tag: [Kotlin, ktor]
---

# CSS 파일 적용하기

앞서 만들었던 `FreeMaker` 템플릿 기반 샘플 예제에서, `css` 파일을 적용하려면 다음과 같이 수정합니다.

## BlogApp.kt

먼저 `routing` 정보에 `static` 정보를 포함하도록 선언합니다.

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
        static("/static") {
            resources("static")
        }

        get("/html-freemarker") {
            call.respond(FreeMarkerContent("index.ftl", mapOf("data" to IndexData(listOf(1, 2, 3))), ""))
        }
    }
}
</pre>

<br>

## css 파일 추가

`resources/static/styles.css` 파일을 추가합니다.

<pre class="prettyprint">
body {
    background: #B9D8FF;
}
</pre>

<br>

## index.ftl 수정

`resources/templates/index.ftl`에 `css` 파일을 불러오는 코드를 추가합니다.

<pre class="prettyprint">
&lt;html&gt;
    &lt;head&gt;
        &lt;link rel="stylesheet" href="/static/styles.css"&gt;
    &lt;/head&gt;

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

<br>

## 그 외

위에서 사용했던 `static` 코드는 텍스트 파일만 추가하는 것이 아니라 이미지 파일이나 기타 다른 파일들도 불러올 수 있습니다.
`<img src="...">` 태그를 이용해서 자유롭게 이미지를 삽입할 수 있습니다.
