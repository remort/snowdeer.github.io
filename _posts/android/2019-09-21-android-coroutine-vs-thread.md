---
layout: post
title: 코루틴(Coroutine) vs 쓰레드(Thread)
category: Android
tag: [Android, Kotlin]
---

# Coroutine is 'Light-weight Thread'

흔히 코루틴은 `Light-weight Thread`라고 합니다. 이 말이 정확히 어떤 것을 의미하는지 좀 더 자세히 알아보도록 하겠습니다.

<br>

## 동시성(Concurrency)과 병렬성(Parallelism)

* 동시성(Concurrency): 다수의 Task를 수행하기 위해서 각 Task를 조금씩 나누어서 실행하는 시분할 방식
* 병렬성(Parallelism): 다수의 Task를 동시에 실행하는 것

동시성은 각 Task를 조금씩 나누어서 실행하는 것이기 때문에 총 실행 시간은 각 Task의 실행 시간을 합친 것과 같습니다. 
예를 들어 10분짜리 Task 5개를 실행한다면, 총 수행 시간은 5 x 10 = 50분이 됩니다. 여기에 각 Task간 작업 전환을 위한
`Context Swithcing`이 추가로 발생합니다.

병렬성은 Task간 전환이 없기 때문에 Context Switching이 발생하지 않습니다. 대신 자원이 Task 수 만큼 필요합니다.
총 수행 시간은 가장 시간이 긴 Task 만큼 소요됩니다.

<br>

## Coroutine & Thread for Concurrency

Coroutine과 Thread는 둘 다 동시성을 보장하기 위한 기술입니다. 
Thread는 OS 레벨에서 각 작업의 동시성을 위해 `Preemtive Scheduling`을 해서 각 작업을 조금씩 나누어서 실행합니다.
Coroutine도 동시성을 목표로 하고 있지만, 각 작업에 Thread를 할당하는 것이 아니라 작은 Object 만을 할당한 다음
이 Object를 스위칭하면서 `Context Switching` 비용을 최대한 줄였습니다. 그래서 `Light-weight Thread`라고 부릅니다.

<br>

## Thread

* Thread는 각 Task 마다 Thread를 할당합니다. 
* 각 Thread는 자체적인 `Stack` 메모리를 가지며 JVM Stack 영역을 가집니다.
* OS 커널에서 `Context Switching`을 해서 동시성을 보장합니다.
* 만약 복수의 Thread를 사용해서 `Thread 1`이 `Thread 2`의 결과를 기다려야 한다면, `Thread 1`은 그 때까지 Blocking 되어 해당 자원을 사용할 수 없습니다.

<br>

## Coroutine

* Task 마다 각각 Object를 할당합니다.
* 각 Coroutine Object는 JVM Heap에 적재됩니다.
* 커널 레벨의 `Context Switching`이 아니라 프로그래머가 컨트롤하는 `Switching`을 통해 동시성을 보장합니다.
* `Task 1` 작업을 수행하다가 `suspend` 되더라도, 해당 Thread 는 유효하기 때문에 `Task 2`를 같은 Thread에서 실행할 수 있습니다.
* 하나의 Thread에서 다수의 Coroutine Object를 실행할 수 있으며, 이 경우 Coroutine Object 교체만 발생하기 때문에 커널 레벨의 `Context Switching`이 발생하지 않습니다.

만약 여러 Thread에서 다수의 Coroutine을 실행할 경우에는 Thread 전환이 일어날 경우 `Context Switching`이 발생합니다. 
Coroutine의 `No Context Switching` 장점을 살리기 위해서는 하나의 Thread에서 복수의 Coroutine Object를 실행하는 것이 유리합니다.

> Coroutine은 기존의 Thread를 좀 더 작은 단위로 쪼개어 사용할 수 있는 개념입니다.

하나의 Thread에서 복수의 Coroutine이 실행될 경우 각 Thread가 가지는 Stack 메모리 영역도 하나가 되어 메모리 절약이 되며, 
공유 메모리 접근으로 발생할 수 있는 Deadlock 문제도 해결될 수 있습니다.

<br>

## Stackful & Stackless

Coroutine은 크게 `Stackful` 방식과 `Stackless` 방식으로 나눌 수 있습니다. Kotlin의 경우는 `Stackless` 방식이기 때문에 약간의 
기능 제한이 있습니다.

Thread의 경우 자체 Stack 메모리 영역을 가지기 때문에 Stack을 이용해서 함수를 실행하고 관리할 수가 있습니다. 

* `Stackful Coroutine` : 코루틴 내부에서 다른 함수를 호출할 수 있고 값을 리턴하거나 suspend 할 수 있습니다.
* `Stackless Coroutine` : caller에게 항상 무엇인가를 리턴해야 하며, 값을 리턴하거나 `no result yet, I'm supended`를 리턴합니다.

다시 요약하면, `Stackless Coroutine`는 항상 값이나 결과를 리턴해야 하기 때문에 코루틴을 호출한 caller가 그 값을 이용해서 판단 및 제어를 해야 하고,
`Stackful Coroutine`는 일반적인 Thread처럼 스스로 suspend도 할 수 있으며 값을 리턴할 수도 있습니다. 

각 언어별 지원하는 코루틴 정보는 다음과 같습니다.

* Stackful Coroutine: Javaflow, Quasar
* Stackless Coroutine: Kotlin, Scala, C#

<br>

## suspend

하나의 Thread는 여러 개의 Coroutine을 실행할 수 있습니다. 이 때 `Context Switching`이 발생하지 않기 때문에 `Light-weight Thread`라고 부릅니다.
하나의 Thread에서 여러 개의 Coroutine가 실행될 때, 실행 중이던 하나의 코루틴이 suspend(멈춤) 상태가 되면, 해당 Thread에서는 해당 Thread 내의 
resume할 다른 코루틴을 찾습니다.

따라서 코루틴 내에서 호출하는 멈출 수 있는 함수는 `suspend` 키워드를 이용해서 선언할 수 있습니다. 

<pre class="prettyprint">
suspend fun getDataFromServer() : Data {
    // TODO
}
</pre>

위와 같은 함수는 `suspend function`이 호출되는 순간 해당 코루틴을 잠시 중단시켜놓을 수 있으며 결과 값이 오면 해당 함수를 다시 resume 시킵니다.
