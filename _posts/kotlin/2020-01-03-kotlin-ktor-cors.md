---
layout: post
title: Kotlin Ktor CORS 설정
category: Kotlin
tag: [Kotlin, ktor]
---

# CORS 설정

<pre class="prettyprint">
fun Application.module() {
    install(CORS) {
        method(HttpMethod.Options)
        method(HttpMethod.Get)
        method(HttpMethod.Post)
        method(HttpMethod.Put)
        method(HttpMethod.Delete)
        method(HttpMethod.Patch)
        header(HttpHeaders.Authorization)
        allowCredentials = true
        anyHost()
    }
    // ...
}
</pre>