---
layout: post
title: Kotlin gRPC 예제 - (1) gRPC 소개
category: Kotlin
tag: [Kotlin, gRPC]
---

# gRPC

`gRPC`는 구글이 공개한 RPC(Remote Procedure Call) 오픈 소스이며, [CNCF](https://www.cncf.io/projects/) 차원에서 밀고 있습니다.
[여기](https://grpc.io/docs/guides/)에서 더 많은 정보를 볼 수 있습니다.

<br>

## HTTP/2의 특징

`HTTP/2` 기반으로 되어 있으며 `HTTP/1`에 비해 다음과 같은 장점을 가집니다.

* HTTP Connection 재사용: 기존 `HTTP`에서는 매 요청마다 Connection을 새로 가져가지만, `gRPC`에서는 `Channel`이라는 형태로
기존 Connection을 유지해서 가져갑니다. 덕분에 매번 Connection하는 Cost가 대폭 줄었습니다.
* 멀티플렉싱: `gRPC`는 하나의 Connection에서 여러 요청을 보낼 수 있습니다. 또한 전송하는 데이터의 우선 순위를 정할 수도 있습니다.
* 메시지 압축: `HTTP/2`의 헤더 압축 기능을 사용합니다.
* 서버에서 Push 가능: 한 번 Connection이 맺어진 다음부터는 양방향 통신이 되기 때문에 `Push` 기능을 자연스럽게 사용할 수 있습니다.

<br>

## gRPC와 Protobuf

gRPC는 메시지를 전송하는 IDL(Interface Definition Language)을 [`Protobuf`](https://developers.google.com/protocol-buffers)라는
라이브러리를 사용하고 있습니다. 

<pre class="prettyprint">
message Person {
  string name = 1;
  int32 id = 2;
  bool has_ponycopter = 3;
}
</pre>

Protobuf는 위와 같은 형식으로 되어 있으며, `.proto` 확장자를 가집니다. `protoc`라는 컴파일러를 이용해서 메시지를 컴파일 할 수 있으며,
컴파일된 결과물로 서버측과 클라이언트측에서 사용할 수 있는 코드가 생성됩니다.

위 메시지 예제에서 `name` 이나 `id`와 같은 필드는 각 언어별로 적절한 Setter/Getter 함수를 자동으로 생성해서 제공해줍니다.

<br>

<pre class="prettyprint">
service Greeter {
  rpc SayHello (HelloRequest) returns (HelloReply) {}
}

message HelloRequest {
  string name = 1;
}

message HelloReply {
  string message = 1;
}
</pre>

위 코드를 보면 `service`와 `message`가 존재합니다. `service`는 서버와 클라이언트 양측에서 사용할 함수들의 묶음이라 생각할 수 있으며,
`message`는 실제로 주고받는 데이터입니다.