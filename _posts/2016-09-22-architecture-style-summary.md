---
layout: post
title: Architecture Style 장단점들
category: 소프트웨어 설계
tag: [architecture style]
---

각 Architecture Style의 장단점들만 간단히 정리해보았습니다.

<br>

## Batch Sequential Architecture Style

장점

* Simple divisions on subsystems
* Each subsystem can be a stand-alone progrma working on input data and producing output data

단점

* Implementation requires external control
* It does not provide interactive interface
* Concurrency is not supported and hence throughput remains low
* High latency

<br>

## Pipe and Filter Architecture Style

장점

* Concurrency : It provides high overall throughput for excessive data processing.
* Reusability : Encapsulation of filters makes it easy to plug and play, and to substitute
* Modifiability : It features low coupling between filters, less impact from adding new filters, and modifying the implementation of any existing filters as long as the I/O interfaces are unchanged.
* Simplicity : It offers clear division between any two filters connected by a pipe.
* Flexibility : It supports both sequential and parallel execution.

단점

* Dynamic Interaction 용으로는 적합하지 않음
* Filter간 데이터 전송 및 Parsing 오버헤드
