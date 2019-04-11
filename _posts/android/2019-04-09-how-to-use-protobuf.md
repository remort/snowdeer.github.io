---
layout: post
title: Protobuf 사용하는 예제
category: Android
tag: [Android]
---

Android에서 [Protobuf](https://developers.google.com/protocol-buffers/)를 사용하는 방법입니다.

<br>

## build.gradle(프로젝트)

먼저 프로젝트의 `build.gradle`에 다음과 같은 세팅을 합니다.

<pre class="prettyprint">
buildscript {
    ext.protobufVersion = '0.8.6'

    dependencies {
        classpath "com.google.protobuf:protobuf-gradle-plugin:$protobufVersion"
    }
}
</pre>

<br>

## build.gradle(모듈)

먼저 플러그인(plugin) 선언을 합니다.

<pre class="prettyprint">
apply plugin: 'com.google.protobuf'
</pre>

그리고 `build.gradle` 파일 가장 아래쪽에 다음과 같은 코드를 입력합니다.

<pre class="prettyprint">
protobuf {
    protoc {
        artifact = 'com.google.protobuf:protoc:3.0.0'
    }
    plugins {
        javalite {
            artifact = 'com.google.protobuf:protoc-gen-javalite:3.0.0'
        }
    }
    generateProtoTasks {
        all().each { task ->
            task.builtins {
                remove java
            }
            task.plugins {
                javalite {}
            }
        }
    }
}
</pre>

그리고 아래 항목도 추가합니다.

<pre class="prettyprint">
dependencies {
    implementation 'com.google.protobuf:protobuf-lite:3.0.0'
}
</pre>

<br>

## proto 파일 정의

`src/main` 아래로 가보면 기본적으로 `java`, `res` 라는 디렉토리가 존재합니다. 여기에 `proto`라는 디렉토리를 하나 만들어줍니다.
그리고 샘플 proto 파일을 하나 만듭니다. 저는 `snowdeer_sample.proto`라는 이름으로 만들었습니다.

Android Studio의 ProjectView에서 보기 모드가 `Android`로 되어 있으면 해당 위치가 제대로 보이지 않기 때문에 `Project` 등으로 변경해줍시다.

### snowdeer_sample.proto

<pre class="prettyprint">
syntax = "proto2";

package com.snowdeer.sample;

option java_package = "com.snowdeer.sample.proto";
option java_outer_classname = "SampleProto";

message User {
  required int32 id = 1;
  required string name = 2;
  optional string address = 3;

  enum PhoneType {
    UNKNOWN = 0;
    MOBILE = 1;
    HOME = 2;
  }

  message PhoneNumber {
    required string number = 1;
    optional PhoneType type = 2 [default = UNKNOWN];
  }

  repeated PhoneNumber phoneNumbers = 4;
}

message UserList {
  repeated User users = 1;
}
</pre>

<br>

## 빌드

이제 모듈 빌드를 해봅니다. 빌드하고 나면 `build/generated/source/...` 디렉토리 아래에 위에서 정의한 이름(예제에서는 `SampleProto`)의 Java 파일이 
생성된 것을 확인할 수 있습니다.

이제 Activity에서는 다음과 같이 사용을 해볼 수 있습니다.

<pre class="prettyprint">
package com.snowdeer.protobufexample;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import com.snowdeer.sample.proto.SampleProto.User;
import com.snowdeer.sample.proto.SampleProto.User.PhoneNumber;
import com.snowdeer.sample.proto.SampleProto.User.PhoneType;

public class MainActivity extends AppCompatActivity {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    PhoneNumber pn = PhoneNumber.newBuilder()
        .setType(PhoneType.MOBILE)
        .setNumber("010-1111-2222")
        .build();

    User user = User.newBuilder()
        .setId(100)
        .setName("snowdeer")
        .setAddress("Seoul")
        .addPhoneNumbers(pn)
        .build();

    Log.i("snowdeer", "[snowdeer] user: " + user.toString());
  }
}
</pre>

<br>

## 실행 결과

<pre class="prettyprint">
   address: "Seoul"
    id: 100
    name: "snowdeer"
    phone_numbers {
      number: "010-1111-2222"
      type: MOBILE
    }
</pre>