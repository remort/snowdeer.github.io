---
layout: post
title: Kotlin에서 실행가능한 jar 생성하기
category: Kotlin
tag: [Kotlin]
---

## Executable jar 생성하기

터미널에서 `java -jar` 명령어를 이용해서 간단하게 실행할 수 있는 `jar` 파일을 생성하는 예제입니다.

<br>

## Main.kt

여기서는 `com.snowdeer`라는 패키지를 만들고 그 안에 `Main.kt` 파일을 생성했습니다.

<pre class="prettyprint">
package com.snowdeer

import java.lang.Thread.sleep

fun main(args: Array<String>) {
    println("Hello SnowDeer.")

    val t = Thread() {
        while (true) {
            println("Thread is Running ...")
            sleep(2000)
        }
    }

    t.start()
}
</pre>

<br>

## build.gradle

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.61'

    repositories {
        mavenCentral()
    }
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

apply plugin: 'java'
apply plugin: 'kotlin'

group 'com.snowdeer'
version '1.0-SNAPSHOT'

sourceCompatibility = 1.8

repositories {
    mavenCentral()
}

dependencies {
    compile "org.jetbrains.kotlin:kotlin-stdlib-jdk8:$kotlin_version"
    testCompile group: 'junit', name: 'junit', version: '4.12'
}

compileKotlin {
    kotlinOptions.jvmTarget = "1.8"
}
compileTestKotlin {
    kotlinOptions.jvmTarget = "1.8"
}

jar {
    manifest {
        attributes 'Main-Class': 'com.snowdeer.MainKt'
    }
    archiveName 'helloKotlin.jar'
    from { configurations.compile.collect { it.isDirectory() ? it : zipTree(it) } }
}
</pre>

위에서 `dependencies` 구문 안에 `compile` 키워드를 확인해야 합니다. 안드로이드 스튜디오에서 사용하던 `gradle`에서는
`compile` 명령어가 `deprecated`되어서 `impelementation` 키워드를 대신 사용하라고 되어 있지만, 사용하는 `gradle` 버전에
따라서는 `compile`만 사용해야 정상 동작되는 경우가 있습니다.

<br>

## 빌드 및 실행

이후 터미널에서 다음 명령어를 이용해서 빌드 및 실행을 해봅니다.

~~~
$ ./gradlew clean build

$ java -jar build/libs/helloKotlin.jar
~~~

<br>

## application 플러그인 추가

만약 `gradle` 명령어를 이용해서 실행을 해보고 싶으면 다음과 같이 `apply plugin: 'application'`와 `mainClassName = "com.snowdeer.MainKt"`를
추가해줍니다.

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.61'

    repositories {
        mavenCentral()
    }
    dependencies {
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

apply plugin: 'java'
apply plugin: 'kotlin'
apply plugin: 'application'

group 'com.snowdeer'
version '1.0-SNAPSHOT'

mainClassName = "com.snowdeer.MainKt"
sourceCompatibility = 1.8

repositories {
    mavenCentral()
}

dependencies {
    compile "org.jetbrains.kotlin:kotlin-stdlib-jdk8:$kotlin_version"
    testCompile group: 'junit', name: 'junit', version: '4.12'
}

compileKotlin {
    kotlinOptions.jvmTarget = "1.8"
}
compileTestKotlin {
    kotlinOptions.jvmTarget = "1.8"
}

jar {
    manifest {
        attributes 'Main-Class': 'com.snowdeer.MainKt'
    }
    archiveName 'helloKotlin.jar'
    from { configurations.compile.collect { it.isDirectory() ? it : zipTree(it) } }
}
</pre>

그 이후 터미널에서 

~~~
$ ./gradlew run
~~~

명령어를 이용해서 바로 실행도 가능합니다.

<br>

## IntelliJ에서 설정하는 방법

`intelliJ`에서 IDE의 `run` 메뉴를 통해 실행하는 방법은 다음과 같습니다.

* 메뉴의 `Run > Run`을 누른다음 `Edit Configuration` 선택
* 왼쪽의 `+` 버튼을 눌러서 `Gradle` 항목 선택
* `Gradle project` 항목 옆의 버튼을 눌러서 실행할 프로젝트 선택
* `Arguments`에 `run` 명령어 추가

또는 다음과 같이 할 수도 있습니다.

* 메뉴의 `Run > Run`을 누른다음 `Edit Configuration` 선택
* 왼쪽의 `+` 버튼을 눌러서 `Application` 항목 선택
* `Main Class`를 선택. 위 예제에서는 `com.snowdeer.MainKt` 선택
* `use classpath of module`에서 실행할 모듈 선택