---
layout: post
title: Kotlin gRPC 예제 - (4) gRPC Server/Client 예제 (Single Request -> Multi Response)
category: Kotlin
tag: [Kotlin, gRPC]
---

# hello.proto

<pre class="prettyprint">
syntax = "proto3";

package com.snowdeer;
option java_outer_classname = "Hello";

service HelloService {
    rpc SayHello (HelloRequest) returns (HelloResponse);
    rpc LotsOfReplies (HelloRequest) returns (stream HelloResponse);
    rpc LotsOfGreetings (stream HelloRequest) returns (HelloResponse);
    rpc BidiHello (stream HelloRequest) returns (stream HelloResponse);
}

message HelloRequest {
    string greeting = 1;
}

message HelloResponse {
    string reply = 1;
}
</pre>

<br>

# HelloServer.kt

Single Request에 대한 Multi Response를 위해 두 번째 메소드인 `lotsOfReplies`에 대해서 구현을 해봅니다.

<pre class="prettyprint">
package com.snowdeer

import io.grpc.ServerBuilder
import io.grpc.stub.StreamObserver

fun main(args: Array&lt;String&gt;) {

    println("[snowdeer] main()")
    val service = HelloService()
    val server = ServerBuilder
        .forPort(10004)
        .addService(service)
        .build()

    println("[snowdeer] server starts()")
    server.start()
    server.awaitTermination()
}

class HelloService : HelloServiceGrpc.HelloServiceImplBase() {

    override fun sayHello(request: Hello.HelloRequest?, responseObserver: StreamObserver&lt;Hello.HelloResponse&gt;?) {
        println("[snowdeer] sayHello(${request?.greeting})")

        val response = Hello.HelloResponse.newBuilder().setReply(request?.greeting).build()
        responseObserver?.onNext(response)
        responseObserver?.onCompleted()
    }

    override fun lotsOfReplies(request: Hello.HelloRequest?, responseObserver: StreamObserver&lt;Hello.HelloResponse&gt;?) {
        println("[snowdeer] lotsOfReplies()")

        for(i in 0 until 5) {
            val resp = Hello.HelloResponse.newBuilder()
                .setReply("hello - $i")
                .build()
            responseObserver?.onNext(resp)

            sleep(1000)
        }
        responseObserver?.onCompleted()
    }

    override fun lotsOfGreetings(responseObserver: StreamObserver&lt;Hello.HelloResponse&gt;?): StreamObserver&lt;Hello.HelloRequest&gt; {
        println("[snowdeer] lotsOfGreetings()")
        return super.lotsOfGreetings(responseObserver)
    }

    override fun bidiHello(responseObserver: StreamObserver&lt;Hello.HelloResponse&gt;?): StreamObserver&lt;Hello.HelloRequest&gt; {
        println("[snowdeer] bidiHello()")
        return super.bidiHello(responseObserver)
    }
}
</pre>

<br>

# HelloClient.kt

<pre class="prettyprint">
package com.snowdeer

import io.grpc.ManagedChannelBuilder

fun main(args: Array&lt;String&gt;) {

    println("[snowdeer] main()")
    val channel = ManagedChannelBuilder
        .forAddress("localhost", 10004)
        .usePlaintext()
        .build()
        
    val stub = HelloServiceGrpc.newBlockingStub(channel)
    val response = stub.lotsOfReplies(getHelloRequest("good morning"))

    response.forEach {
        println("[snowdeer] response: ${it.reply}")
    }

    println("[snowdeer] response.forEach is finished")
}

fun getHelloRequest(greeting: String): Hello.HelloRequest {
    return Hello.HelloRequest.newBuilder()
        .setGreeting(greeting)
        .build()
}
</pre>