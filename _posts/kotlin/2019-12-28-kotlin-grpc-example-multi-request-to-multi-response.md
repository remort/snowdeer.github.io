---
layout: post
title: Kotlin gRPC 예제 - (6) gRPC Server/Client 예제 (Multi Request -> Multi Response)
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

Multi Request에 대한 Single Response를 위해 네 번째 메소드인 `BidiHello`에 대해서 구현을 해봅니다.
지금부터는 Client 쪽 코드에 `BlockingStub`이 아닌 `AsyncStub`을 사용하는 것을 주의합니다.

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
        return object : StreamObserver&lt;Hello.HelloRequest&gt; {

            override fun onNext(value: Hello.HelloRequest?) {
                println("[snowdeer] lotsOfGreetings() - onNext(${value?.greeting})")
            }

            override fun onError(t: Throwable?) {
                println("[snowdeer] lotsOfGreetings() - onError()")
            }

            override fun onCompleted() {
                println("[snowdeer] lotsOfGreetings() - onCompleted()")

                val response = Hello.HelloResponse.newBuilder().setReply("lotsOfGreetings is completed").build()
                responseObserver?.onNext(response)
                responseObserver?.onCompleted()
            }
        }
    }

    override fun bidiHello(responseObserver: StreamObserver&lt;Hello.HelloResponse&gt;?): StreamObserver&lt;Hello.HelloRequest&gt; {
        return object : StreamObserver&lt;Hello.HelloRequest&gt; {

            override fun onNext(value: Hello.HelloRequest?) {
                println("[snowdeer] bidiHello() - onNext(${value?.greeting})")

                val resp = Hello.HelloResponse.newBuilder()
                    .setReply("Response to Client - (${value?.greeting})")
                    .build()
                responseObserver?.onNext(resp)
            }

            override fun onError(t: Throwable?) {
                println("[snowdeer] bidiHello() - onError()")
            }

            override fun onCompleted() {
                println("[snowdeer] bidiHello() - onCompleted()")
                responseObserver?.onCompleted()
            }
        }
    }
}
</pre>

위 코드에서 `lotsOfGreetings` 메소드가 리턴하는 것은 `Hello.HelloRequest`를 처리하는 `StreamObserver` 객체인 것을 알 수 있습니다.

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
    val asyncStub = HelloServiceGrpc.newStub(channel)

    val requestObserver = asyncStub.bidiHello(ResponseStreamObserver())

    for(i in 0 until 10) {
        requestObserver.onNext(getHelloRequest("bidiHello to Server - $i"))
        sleep(800)
    }
    requestObserver.onCompleted()

    while(true) {
        sleep(1000)
        println("Thread is running.")
    }
}

fun getHelloRequest(greeting: String): Hello.HelloRequest {
    return Hello.HelloRequest.newBuilder()
        .setGreeting(greeting)
        .build()
}

class ResponseStreamObserver : StreamObserver&lt;Hello.HelloResponse&gt; {
    override fun onNext(value: Hello.HelloResponse?) {
        println("[snowdeer] onNext(${value?.reply})")
    }

    override fun onError(t: Throwable?) {
        println("[snowdeer] onError()")
    }

    override fun onCompleted() {
        println("[snowdeer] onCompleted()")
    }
}
</pre>