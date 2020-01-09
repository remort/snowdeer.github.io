---
layout: post
title: Kotlin Ktor 복수 클라이언트를 지원하는 WebSocket 예제
category: Kotlin
tag: [Kotlin, ktor]
---

# 복수 클라이언트를 지원하는 WebSocket 예제

<br>

## Main.kt

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.http.cio.websocket.DefaultWebSocketSession
import io.ktor.http.cio.websocket.Frame
import io.ktor.http.cio.websocket.readText
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.response.respondFile
import io.ktor.routing.get
import io.ktor.routing.routing
import io.ktor.websocket.WebSockets
import io.ktor.websocket.webSocket
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import java.io.File
import java.time.Duration
import java.util.*
import kotlin.collections.LinkedHashSet

private val wsConnections = Collections.synchronizedSet(LinkedHashSet&lt;DefaultWebSocketSession&gt;())

fun Application.main() {

    install(WebSockets) {
        pingPeriod = Duration.ofSeconds(60) // Disabled (null) by default
        timeout = Duration.ofSeconds(15)
        maxFrameSize = Long.MAX_VALUE // Disabled (max value). The connection will be closed if surpassed this length.
        masking = false
    }

    routing {
        static("/static") {
            resources("static")
        }

        get("/") {
            val file = File("src/main/resources/static/index.html")
            call.respondFile(file)
        }

        webSocket("/chat") {
            println("[snowdeer] chat starts")
            wsConnections += this
            try {
                while (true) {
                    when (val frame = incoming.receive()) {
                        is Frame.Text -> {
                            val text = frame.readText()

                            for (conn in wsConnections) {
                                conn.outgoing.send(Frame.Text(text))
                            }
                        }
                    }
                }
            } finally {
                wsConnections -= this
            }

            println("[snowdeer] chat is finished")
        }
    }

    startCoroutine()
}

private fun startCoroutine() {
    println("[snowdeer] startCoroutine()")

    GlobalScope.launch {
        var count = 0L
        while (true) {
            println("Coroutine is running...")

            count++
            for (conn in wsConnections) {
                conn.outgoing.send(Frame.Text("hello($count)"))
            }

            delay(1000)
        }
    }
}
</pre>
