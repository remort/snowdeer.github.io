---
layout: post
title: Kotlin Ktor WebSocket 사용하기
category: Kotlin
tag: [Kotlin, ktor]
---

# Ktor WebSocket 사용하기

`WebSocket`를 이용하면 실시간 양방향 통신을 할 수 있습니다.

<br>

## build.gradle

먼저 `build.gradle`에 다음 라이브러리를 추가해줍니다.

<pre class="prettyprint">
dependencies {
    implementation "io.ktor:ktor-websockets:$ktor_version"
}
</pre>

<br>

## Main.kt

`routing`은 다음 코드처럼 할 수 있으며, `incoming.receive()` 메소드와 `outgoing.send(Frame.Text(text)` 메소드는 블럭킹(Blocking) 메소드입니다.

<pre class="prettyprint">
package com.snowdeer

import io.ktor.application.Application
import io.ktor.application.install
import io.ktor.http.cio.websocket.Frame
import io.ktor.http.cio.websocket.readText
import io.ktor.http.content.resources
import io.ktor.http.content.static
import io.ktor.routing.routing
import io.ktor.websocket.WebSockets
import io.ktor.websocket.webSocket
import java.time.Duration

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

## client.html

다음 코드를 브라우저에서 실행해서 WebSocket 통신이 잘 되는지 확인할 수 있습니다.

<pre class="prettyprint">
&lt;!DOCTYPE HTML&gt;

&lt;html&gt;
   &lt;head&gt;
      
      &lt;script type = "text/javascript"&gt;
         function startWebSocket() {
            
            if ("WebSocket" in window) {
               alert("You can use WebSocket.");
               
               var ws = new WebSocket("ws://localhost:8080/chat");
				
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