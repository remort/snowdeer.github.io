---
layout: post
title: 코루틴(Coroutine) 간략 설명
category: Android
tag: [Android, Kotlin]
---

# 코루틴의 구성 요소

코틀린은 크게 다음과 같은 요소들로 구성되어 있습니다.

* CoroutineScope
* CoroutineContext
* Dispatcher

<br>

## CoroutineScope

`CoroutineScope`는 코루틴이 실행되는 범위입니다. 간편하게 사용하는 `GlobalScope`의 경우 `CoroutineScope`의 한 종류입니다.

<br>

## CoroutineContext

`CoroutineContext`는 안드로이드에서의 `Context`와 비슷한 역할을 한다고 생각하면 됩니다. 코루틴을 실행하는 제어 정보가 들어있으며,
주요 요소로는 `Job`이나 `Dispatcher`가 있습니다. 코루틴을 사용하다보면 대부분 `CoroutineScope`과 `Dispatcher` 위주로 사용하다보니,
`CoroutineContext`는 직접적으로 사용하는 경우는 적을 수도 있습니다.

<br>

## Dispatcher

어떤 Thread를 어떻게 동작할 것인지 선택할 수 있습니다. 미리 준비된 Dispatcher 들을 제공하고 있으며 대표적인 예는 다음과 같습니다.

* `Dispatchers.Default`: CPU 사용량이 많아서 별도 thread로 분리해야 하는 경우입니다.
* `Dispatchers.IO`: Network이나 File 등을 처리할 때 사용합니다.
* `Dispatchers.Main`: 기본적으로 사용하지 못하는데, 안드로이드의 경우는 사용 가능합니다. Main UI Thread 에서 동작하도록 할 수 있습니다.

<br>

## 코루틴의 사용 방법

1. 어떤 Dispatcher를 사용할 것인지 정합니다.
2. 위에서 정한 Dispatcher를 이용해서 CoroutineScope를 생성합니다.
3. Coroutine의 `launch`나 `async`에 동작할 코드 블럭을 전달합니다. `launch`는 `Job`, `async`는 `Deferred` 객체를 리턴합니다.
4. 위에서 리턴한 `Job`이나 `Deffered` 객체를 제어합니다. `cancel`, `join` 등으로 코루틴을 제어할 수 있습니다.