---
layout: post
title: Kotlin gRPC 예제 - (3) gRPC Server/Client 예제 (Single Request -> Single Response)
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

위에서 총 4개의 메소드를 정의했지만, 일단 첫 번째 메소드인 `sayHello`에 대해서만 구현을 해봅니다.

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
    val response = stub.sayHello(getHelloRequest("hello. snowdeer"))

    println("[snowdeer] response(${response.reply})")
}

fun getHelloRequest(greeting: String): Hello.HelloRequest {
    return Hello.HelloRequest.newBuilder()
        .setGreeting(greeting)
        .build()
}
</pre>