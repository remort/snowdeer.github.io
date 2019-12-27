---
layout: post
title: Kotlin gRPC 예제 - (2) gRPC를 사용하기 위한 build.gradle 및 Protobuf 메시지 빌드
category: Kotlin
tag: [Kotlin, gRPC]
---

# gRPC를 사용하기 위한 build.gradle

## build.gradle

`gRPC`를 사용하기 위해서는 필요한 플러그인과 종속성을 추가해야 합니다.

<pre class="prettyprint">
group 'com.snowdeer'
group 'com.snowdeer'
version '1.0-SNAPSHOT'

apply plugin: 'java'
apply plugin: 'kotlin'
apply plugin: 'application'
apply plugin: 'com.google.protobuf'
apply plugin: 'idea'

mainClassName = "com.snowdeer.MainKt"

repositories {
    mavenCentral()
}

buildscript {
    ext.kotlin_version = '1.3.61'
    ext.grpc_version = '1.17.0'

    repositories {
        mavenCentral()
    }
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
        classpath 'com.google.protobuf:protobuf-gradle-plugin:0.8.8'
    }
}

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk8"

    compile "com.google.api.grpc:proto-google-common-protos:0.1.9"
    compile "io.grpc:grpc-netty:${grpc_version}"
    compile "io.grpc:grpc-protobuf:${grpc_version}"
    compile "io.grpc:grpc-stub:${grpc_version}"

    compile("javax.annotation:javax.annotation-api:1.3.2")

    testCompile group: 'junit', name: 'junit', version:'4.12'
}

idea {
    module {
        sourceDirs += file("${projectDir}/build/generated/source/proto/main/java");
        sourceDirs += file("${projectDir}/build/generated/source/proto/main/grpc");
    }
}

compileKotlin.dependsOn ':generateProto'
sourceSets.main.java.srcDirs += 'build/generated/source/proto/main/grpc'
sourceSets.main.java.srcDirs += 'build/generated/source/proto/main/java'

compileKotlin {
    kotlinOptions.jvmTarget = "1.8"
}
compileTestKotlin {
    kotlinOptions.jvmTarget = "1.8"
}


protobuf {
    protobuf {
        protoc { artifact = "com.google.protobuf:protoc:3.6.1" }
        plugins {
            grpc { artifact = "io.grpc:protoc-gen-grpc-java:${grpc_version}" }
        }
        generateProtoTasks {
            all()*.plugins { grpc {} }
        }
    }
}
</pre>

<br>

## main/proto/hello.proto

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

`HelloService`에는 4개의 메소드가 정의되어 있습니다. 각각은 다음 특징을 가집니다.

* `SayHello`: 일반적인 Message 형태로 단일 요청에 단일 응답
* `LotsOfReplies`: 단일 요청에 대해 Stream 형태의 응답 전달
* `LotsOfGreetings`: 클라이언트에서 서버로 Stream 형태의 요청을 보내며, 서버에서는 단일 응답 전달함
* `BidiHello`: 양방향으로 Stream 형태의 요청과 응답을 전달함

위 파일을 추가한다음 Gradle 빌드 과정을 거치면 `build/generated/source/proto` 디렉토리 아래에 메시지 관련 클래스들이 생성됩니다.
