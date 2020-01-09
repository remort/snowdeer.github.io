---
layout: post
title: Kotlin Ktor HTML 파일 리턴하기
category: Kotlin
tag: [Kotlin, ktor]
---

# Ktor HTML 파일 리턴하기

앞서 작업했던 [WebSocket](/kotlin/2020/01/04/kotlin-ktor-web-socket-example/) 예제를 확장해서
사용합니다.

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
import java.io.File
import java.time.Duration
import java.util.*
import kotlin.collections.LinkedHashSet

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
            while (true) {
                when (val frame = incoming.receive()) {
                    is Frame.Text -> {
                        val text = frame.readText()
                        println("[snowdeer] text: $text")
                        outgoing.send(Frame.Text("$text from Server"))
                    }
                }
            }

            println("[snowdeer] chat is finished")
        }
    }
}
</pre>

<br>

## index.html

`resources/static/index.html` 파일을 생성하고 아래와 같은 내용을 채웁니다.

<pre class="prettyprint">
&lt;!DOCTYPE HTML&gt;

&lt;html&gt;
   &lt;head&gt;
      
      &lt;script type = "text/javascript"&gt;
         function startWebSocket() {
            
            if ("WebSocket" in window) {
               alert("You can use WebSocket.");
               
               var ws = new WebSocket("ws://" + location.host + "/chat");
				
               ws.onopen = function() {
                  alert("WebSocket is opened.");
                  ws.send("Hello");
                  alert("Send message to Server(Hello).");
               };
				
               ws.onmessage = function (evt) { 
                  var msg = evt.data;
                  alert("Message is received(" + msg + ")");
               };
				
               ws.onclose = function() { 
                  alert("WebSocket is closed."); 
               };
            } else {
               alert("Your browser does not support WebSocket !!!");
            }
         }
      &lt;/script&gt;
		
   &lt;/head&gt;
   
   &lt;body&gt;
      &lt;a href = "javascript:startWebSocket()"&gt;Start WebSocket&lt;/a&gt;
   &lt;/body&gt;
&lt;/html&gt;
</pre>

로컬 서버 주소는 `var ws = new WebSocket("ws://localhost:8080/chat");`으로 사용해도 되고,
`var ws = new WebSocket("ws://" + location.host + "/chat");`으로 사용해도 됩니다.