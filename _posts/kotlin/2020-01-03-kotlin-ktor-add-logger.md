---
layout: post
title: Kotlin Ktor에 Logger 연결하기
category: Kotlin
tag: [Kotlin, ktor]
---

# Ktor에 Logger 연결하기

Ktor에 Logger를 연결하지 않으면, 실행할 때 마다 다음 경고 메시지를 볼 수 있습니다.

~~~
SLF4J: Failed to load class "org.slf4j.impl.StaticLoggerBinder".
SLF4J: Defaulting to no-operation (NOP) logger implementation
SLF4J: See http://www.slf4j.org/codes.html#StaticLoggerBinder for further details.
~~~

경고 메시지라 실행하는데 큰 문제는 없지만, 걸리적거리기도 하고 
원할한 디버깅을 위해 Logger를 설정하는 것을 추천합니다.

기본적으로 제공하는 `Logback provider`를 사용해보도록 하겠습니다.

<br>

## build.gradle

먼저 `build.gradle`에 다음 항목을 추가합니다.

<pre class="prettyprint">
implementation "ch.qos.logback:logback-classic:1.2.3"
</pre>

<br>

## logback.xml

<pre class="prettyprint">
&lt;configuration&gt;
    &lt;appender name="STDOUT" class="ch.qos.logback.core.ConsoleAppender"&gt;
        &lt;encoder&gt;
            &lt;pattern&gt;%d{YYYY-MM-dd HH:mm:ss.SSS} [%thread] %-5level %logger{36} - %msg%n&lt;/pattern&gt;
        &lt;/encoder&gt;
    &lt;/appender&gt;

    &lt;root level="trace"&gt;
        &lt;appender-ref ref="STDOUT"/&gt;
    &lt;/root&gt;

    &lt;logger name="org.eclipse.jetty" level="INFO"/&gt;
    &lt;logger name="io.netty" level="INFO"/&gt;
&lt;/configuration&gt;
</pre>

<br>

그 이후 프로그램을 종료했다가 재실행을 하면 다음과 같은 메시지가 출력될 것입니다.

~~~
2020-01-06 18:47:51.160 [main] TRACE Application - {
    # application.conf @ file:/Users/snowdeer/Workspace/HelloKtor/build/resources/main/application.conf: 6
    "application" : {
        # application.conf @ file:/Users/snowdeer/Workspace/HelloKtor/build/resources/main/application.conf: 7
        "modules" : [
            # application.conf @ file:/Users/snowdeer/Workspace/HelloKtor/build/resources/main/application.conf: 7
            "com.snowdeer.BlogAppKt.main"
        ]
    },
    # application.conf @ file:/Users/snowdeer/Workspace/HelloKtor/build/resources/main/application.conf: 2
    "deployment" : {
        # application.conf @ file:/Users/snowdeer/Workspace/HelloKtor/build/resources/main/application.conf: 3
        "port" : 8080
    },
    # Content hidden
    "security" : "***"
}

2020-01-06 18:47:51.279 [main] INFO  Application - No ktor.deployment.watch patterns specified, automatic reload is not active
2020-01-06 18:47:51.598 [main] INFO  Application - Responding at http://0.0.0.0:8080
~~~

이 상태에서 `GET`, `POST` 등의 메소드를 이용해서 서버에 접속해보면 다양한 로그 메시지들이 출력되는 것을 확인할 수 있습니다.