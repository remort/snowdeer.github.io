---
layout: post
title: JAVA_HOME 환경 변수 설정 (Java 버전 여러 개 설치된 경우)

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# JAVA_HOME 환경 변수 설정

개발을 하다보면 환경 변수 `JAVA_HOME`에 값을 설정해야 할 일이 자주 발생합니다. 
먼저 `/usr/libexec/java_home -V` 명령어를 이용하여 Mac에 설치된 Java 버전을 확인합니다. 만약 별도로 Java를 설치하지 않았으면 다음과 같은 결과가 출력됩니다.

~~~
$ /usr/libexec/java_home -V

Matching Java Virtual Machines (1):
    11.0.1, x86_64:	"OpenJDK 11.0.1"	/Library/Java/JavaVirtualMachines/openjdk-11.0.1.jdk/Contents/Home

/Library/Java/JavaVirtualMachines/openjdk-11.0.1.jdk/Contents/Home
~~~

<br>

맥 OS에 설치된 Java 버전이 2 가지 이상인 경우에도 동일한 방법으로 확인할 수 있습니다. 
저같은 경우는 특정 프레임워크 빌드를 위해 JDK 1.8을 추가로 설치했더니 다음과 같이 출력되었습니다.

~~~
/usr/libexec/java_home -V
Matching Java Virtual Machines (2):
    11.0.1, x86_64:	"OpenJDK 11.0.1"	/Library/Java/JavaVirtualMachines/openjdk-11.0.1.jdk/Contents/Home
    1.8.0_192, x86_64:	"Java SE 8"	/Library/Java/JavaVirtualMachines/jdk1.8.0_192.jdk/Contents/Home

/Library/Java/JavaVirtualMachines/openjdk-11.0.1.jdk/Contents/Home
~~~

이 중 원하는 버전으로 `JAVA_HOME`으로 설정하면 됩니다.

또는 아래의 명령어를 수행하면 보다 편리하게 설정 가능합니다.

~~~
# Java 10
export JAVA_HOME=$(/usr/libexec/java_home -v 10)

# Java 9
export JAVA_HOME=$(/usr/libexec/java_home -v 9)

# Java 1.8
export JAVA_HOME=$(/usr/libexec/java_home -v 1.8)

# Java 1.7
export JAVA_HOME=$(/usr/libexec/java_home -v 1.7)

# Java 1.6
export JAVA_HOME=$(/usr/libexec/java_home -v 1.6)
~~~